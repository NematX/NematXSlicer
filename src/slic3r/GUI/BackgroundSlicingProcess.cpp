///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/ Copyright (c) Prusa Research 2018 - 2023 Lukáš Matěna @lukasmatena, Oleksandra Iushchenko @YuSanka, Vojtěch Bubník @bubnikv, Pavel Mikuš @Godrak, David Kocík @kocikdav, Enrico Turri @enricoturri1966, Tomáš Mészáros @tamasmeszaros, Roman Beránek @zavorka, Vojtěch Král @vojtechkral
///|/ Copyright (c) 2022 ole00 @ole00
///|/ Copyright (c) 2021 Ilya @xorza
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "BackgroundSlicingProcess.hpp"

// Background slicing coordinates the worker and GUI callbacks. FFF exports use
// a request-owned configuration and thumbnail callback; uploads first publish a
// private file, then transfer it to the upload queue after successful export.

#include "GUI_App.hpp"
#include "GUI.hpp"
#include "MainFrame.hpp"
#include "format.hpp"
#include "ExportInputsDialog.hpp"
#include "Field.hpp"
#include "MsgDialog.hpp"
#include "libslic3r/Api/host/ExportInputs.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/GCode/Thumbnails.hpp"

#include <wx/app.h>
#include <wx/panel.h>
#include <wx/stdpaths.h>
#include <wx/weakref.h>

// For zipped archive creation
#include <wx/stdstream.h>
#include <wx/wfstream.h>
#include <wx/zipstrm.h>

#include <miniz.h>

// Print now includes tbb, and tbb includes Windows. This breaks compilation of wxWidgets if included before wx.
#include "libslic3r/Print.hpp"
#include "libslic3r/ScopedTemporaryFile.hpp"
#include "libslic3r/SLAPrint.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/Format/SL1.hpp"
#include "libslic3r/Thread.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/libslic3r.h"

#include <cassert>
#include <optional>
#include <stdexcept>
#include <cctype>

#include <boost/format/format_fwd.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/log/trivial.hpp>
#include <boost/nowide/cstdio.hpp>
#include "I18N.hpp"
#include "RemovableDriveManager.hpp"

#include "slic3r/GUI/Plater.hpp"

#ifdef WIN32
#include <windows.h>
#include "slic3r/StackWalker.h"
#endif

/*
Coordinate background processing between the GUI and the slicing worker.

The GUI supplies the model, configuration and requested action. The worker
performs slicing or export and posts status/completion events; GUI callbacks
update the preview and controls on the UI thread. Stopping joins or suspends
the worker before data needed by its current operation is replaced.

File export and print-host upload share generated print results, but have
different final owners. An upload must first finish a private file, then hand
that file to the upload queue. GUI rendering or input requests must be serviced
on the UI thread without holding the print-state lock while waiting.

Representative direct calls (branches are not a timeline):

    BackgroundSlicingProcess::process_fff()
        |-- m_print->process()
        |-- wxQueueEvent()
        |-- export_gcode()
        |-- finalize_gcode()
        `-- prepare_upload()

    BackgroundSlicingProcess::prepare_upload()
        |-- select_gcode_exporter()
        |-- m_sla_print->export_print()
        `-- enqueue()
*/

namespace Slic3r {

bool SlicingProcessCompletedEvent::critical_error() const
{
	try {
		this->rethrow_exception();
	} catch (const Slic3r::SlicingError &) {
		// Exception derived from SlicingError is non-critical.
		return false;
	} catch (...) {
	}
	return true;
}

bool SlicingProcessCompletedEvent::invalidate_plater() const
{
	if (critical_error())
	{
		try {
			this->rethrow_exception();
		}
		catch (const Slic3r::ExportError&) {
			// Exception thrown by copying file does not ivalidate plater
			return false;
		}
		catch (...) {
		}
		return true;
	}
	return false;
}

std::pair<std::string, bool> SlicingProcessCompletedEvent::format_error_message() const
{
	std::string error;
	bool        monospace = false;
	try {
		this->rethrow_exception();
    } catch (const std::bad_alloc &ex) {
		error = GUI::format(_L("%1% has encountered an error. It was likely caused by running out of memory. "
                              "If you are sure you have enough RAM on your system, this may also be a bug and we would "
                              "be glad if you reported it."), SLIC3R_APP_NAME);
        error += "\n\n" + std::string(ex.what());
#ifdef WIN32
        error += " \nPlease join the 'crash_log.txt' file (if it exists) content to your report.";
#endif
    } catch (const HardCrash &ex) {
        error = GUI::format(_L("%1% has encountered a fatal error: \"%2%\""), SLIC3R_APP_NAME, ex.what())
                + "\n\n"
                + GUI::format(_u8L("Please save your project and restart %1%. "
                     "We would be glad if you reported the issue."), SLIC3R_APP_NAME);
#ifdef WIN32
        error += " \nPlease join the 'crash_log.txt' file (if it exists) content to your report.";
#endif
    } catch (PlaceholderParserError &ex) {
		error = ex.what();
		monospace = true;
    } catch (std::exception &ex) {
		error = ex.what();
	} catch (...) {
		error = "Unknown C++ exception.";
	}
	return std::make_pair(std::move(error), monospace);
}

BackgroundSlicingProcess::BackgroundSlicingProcess()
{
    boost::filesystem::path temp_path(wxStandardPaths::Get().GetTempDir().utf8_str().data());
    temp_path /= (boost::format(".%1%.gcode") % get_current_pid()).str();
	m_temp_output_path = temp_path.string();
}

BackgroundSlicingProcess::~BackgroundSlicingProcess() 
{ 
	this->stop();
	this->join_background_thread();
	boost::nowide::remove(m_temp_output_path.c_str());
}

bool BackgroundSlicingProcess::select_technology(PrinterTechnology tech)
{
	bool changed = false;
	if (m_print == nullptr || m_print->technology() != tech) {
		if (m_print != nullptr)
			this->reset();
		switch (tech) {
		case ptFFF: m_print = m_fff_print; break;
		case ptSLA: m_print = m_sla_print; break;
        case ptSLS: m_print = m_fff_print; break;
        default: assert(false); break;
		}
		changed = true;
	}
	assert(m_print != nullptr);
	return changed;
}

PrinterTechnology BackgroundSlicingProcess::current_printer_technology() const
{
	return m_print->technology();
}

std::string BackgroundSlicingProcess::output_filepath_for_project(const boost::filesystem::path &project_path)
{
	assert(m_print != nullptr);
    if (project_path.empty())
        return m_print->output_filepath("");
    return m_print->output_filepath(project_path.parent_path().string(), project_path.stem().string());
}

// Resolve borrowed project and suffix strings through PrintBase; return an owned FFF path, preserve SLA resolution, and propagate placeholder or filesystem failures.
std::string BackgroundSlicingProcess::output_filepath_for_project(
    const boost::filesystem::path &project_path,
    const std::string &default_extension)
{
    assert(m_print != nullptr);
    if (m_print != m_fff_print)
        return this->output_filepath_for_project(project_path);
    if (project_path.empty())
        return m_print->output_filepath(std::string(), std::string(), default_extension);
    return m_print->output_filepath(project_path.parent_path().string(),
                                    project_path.stem().string(),
                                    default_extension);
}

// Run m_fff_print, publish m_temp_output_path for preview, finalize a queued file export or upload, and propagate print, exporter, and cancellation failures.
void BackgroundSlicingProcess::process_fff()
{
	assert(m_print == m_fff_print);
	m_print->process();
	wxCommandEvent evt(m_event_slicing_completed_id);
	// Post the Slicing Finished message for the G-code viewer to update.
	// Passing the timestamp 
	evt.SetInt((int)(m_fff_print->step_state_with_timestamp(STEP_FINALIZE_SLICING).timestamp));
	wxQueueEvent(GUI::wxGetApp().mainframe->m_plater, evt.Clone());
	// The preview needs only the G-code reference: thumbnails are rendered by the export, when a file is written.
	Orchestrator::instance().export_gcode(*m_fff_print, m_temp_output_path, m_gcode_result);
	if (this->set_step_started(bspsGCodeFinalize)) {
	    if (m_fff_export_scheduled) {
			wxQueueEvent(GUI::wxGetApp().mainframe->m_plater, new wxCommandEvent(m_event_export_began_id));
			finalize_gcode();
	    } else if (! m_upload_job.empty()) {
			wxQueueEvent(GUI::wxGetApp().mainframe->m_plater, new wxCommandEvent(m_event_export_began_id));
			prepare_upload();
	    } else {
			//m_print->set_status(100, _u8L("Slicing complete"));
	    }
		this->set_step_done(bspsGCodeFinalize);
	}
	evt.SetInt((int)(m_fff_print->step_state_with_timestamp(STEP_GCODE).timestamp));
	wxQueueEvent(GUI::wxGetApp().mainframe->m_plater, evt.Clone());
}

void BackgroundSlicingProcess::process_sla()
{
    assert(m_print == m_sla_print);
    m_print->process();
	wxCommandEvent evt(m_event_slicing_completed_id);
	evt.SetInt((int)(m_sla_print->step_state_with_timestamp(SLAPrintStep::slapsRasterize).timestamp));
	wxQueueEvent(GUI::wxGetApp().mainframe->m_plater, evt.Clone());
    if (this->set_step_started(bspsGCodeFinalize)) {
        if (! m_export_path.empty()) {
			wxQueueEvent(GUI::wxGetApp().mainframe->m_plater, new wxCommandEvent(m_event_export_began_id));

        	const std::string export_path = m_sla_print->print_statistics().finalize_output_path(m_export_path);

            ThumbnailsParams params{current_print()->full_print_config().option<ConfigOptionPoints>("thumbnails")->get_values(), true, true, true, true};
            apply_thumbnail_colors(params, current_print()->full_print_config());
            ThumbnailsList thumbnails = this->render_thumbnails(params);
			m_sla_print->export_print(export_path, thumbnails);

        } else if (! m_upload_job.empty()) {
			wxQueueEvent(GUI::wxGetApp().mainframe->m_plater, new wxCommandEvent(m_event_export_began_id));
            prepare_upload();
        } else {
			m_print->set_status(100, _u8L("Slicing complete"));
        }
        this->set_step_done(bspsGCodeFinalize);
    }
}

void BackgroundSlicingProcess::thread_proc()
{
    set_current_thread_name("slic3r_BgSlcPcs");
    name_tbb_thread_pool_threads_set_locale();

    // Set "C" locales and enforce OSX QoS level on all threads entering an arena.
    // The cost of the callback is quite low: The callback is called once per thread
    // entering a parallel loop and the callback is guarded with a thread local
    // variable to be executed just once.
	TBBLocalesSetter setter;

	assert(m_print != nullptr);
	assert(m_print == m_fff_print || m_print == m_sla_print);
	std::unique_lock<std::mutex> lck(m_mutex);
	// Let the caller know we are ready to run the background processing task.
	m_state = STATE_IDLE;
	lck.unlock();
	m_condition.notify_one();
	for (;;) {
		//assert(m_state == STATE_IDLE || m_state == STATE_CANCELED || m_state == STATE_FINISHED);
		// Wait until a new task is ready to be executed, or this thread should be finished.
		lck.lock();
		m_condition.wait(lck, [this](){ return m_state == STATE_STARTED || m_state == STATE_EXIT; });
		if (m_state == STATE_EXIT)
			// Exiting this thread.
			break;
		// Process the background slicing task.
		m_state = STATE_RUNNING;
		lck.unlock();
		std::exception_ptr exception;
#ifdef _WIN32
		this->call_process_seh_throw(exception);
#else
		this->call_process(exception);
#endif
		m_print->finalize();
		lck.lock();
		m_state = m_print->canceled() ? STATE_CANCELED : STATE_FINISHED;
		if (m_print->cancel_status() != Print::CANCELED_INTERNAL) {
			// Only post the canceled event, if canceled by user.
			// Don't post the canceled event, if canceled from Print::apply().
			SlicingProcessCompletedEvent evt(m_event_finished_id, 0, 
				(m_state == STATE_CANCELED) ? SlicingProcessCompletedEvent::Cancelled :
				exception ? SlicingProcessCompletedEvent::Error : SlicingProcessCompletedEvent::Finished, exception);
        	wxQueueEvent(GUI::wxGetApp().mainframe->m_plater, evt.Clone());
        	// Cancelled by the user, not internally. The FFF Print keeps the partial
        	// outputs until the next run removes them before resuming.
        }
	    m_print->restart();
		lck.unlock();
		// Let the UI thread wake up if it is waiting for the background task to finish.
	    m_condition.notify_one();
	    // Let the UI thread see the result.
	}
	m_state = STATE_EXITED;
	lck.unlock();
	// End of the background processing thread. The UI thread should join m_thread now.
}

#ifdef _WIN32
// Only these SEH exceptions will be catched and turned into Slic3r::HardCrash C++ exceptions.
static bool is_win32_seh_harware_exception(unsigned long ex) throw() {
	return
		ex == STATUS_ACCESS_VIOLATION ||
		ex == STATUS_DATATYPE_MISALIGNMENT ||
		ex == STATUS_FLOAT_DIVIDE_BY_ZERO ||
		ex == STATUS_FLOAT_OVERFLOW ||
		ex == STATUS_FLOAT_UNDERFLOW ||
#ifdef STATUS_FLOATING_RESEVERED_OPERAND
		ex == STATUS_FLOATING_RESEVERED_OPERAND ||
#endif // STATUS_FLOATING_RESEVERED_OPERAND
		ex == STATUS_ILLEGAL_INSTRUCTION ||
		ex == STATUS_PRIVILEGED_INSTRUCTION ||
		ex == STATUS_INTEGER_DIVIDE_BY_ZERO ||
		ex == STATUS_INTEGER_OVERFLOW ||
		ex == STATUS_STACK_OVERFLOW;
}

// Rethrow some SEH exceptions as Slic3r::HardCrash C++ exceptions.
static void rethrow_seh_exception(unsigned long win32_seh_catched)
{
	if (win32_seh_catched) {
		// Rethrow SEH exception as Slicer::HardCrash.
		if (win32_seh_catched == STATUS_ACCESS_VIOLATION)
			throw Slic3r::HardCrash(_u8L("Access (status) violation"));
		if (win32_seh_catched == STATUS_DATATYPE_MISALIGNMENT)
			throw Slic3r::HardCrash(_u8L("Access violation (misalignement)"));
		if (win32_seh_catched == STATUS_ILLEGAL_INSTRUCTION || win32_seh_catched == STATUS_PRIVILEGED_INSTRUCTION)
			throw Slic3r::HardCrash(_u8L("Illegal instruction"));
		if (win32_seh_catched == STATUS_FLOAT_DIVIDE_BY_ZERO || win32_seh_catched == STATUS_INTEGER_DIVIDE_BY_ZERO)
			throw Slic3r::HardCrash(_u8L("Divide by zero"));
		if (win32_seh_catched == STATUS_FLOAT_OVERFLOW || win32_seh_catched == STATUS_INTEGER_OVERFLOW)
			throw Slic3r::HardCrash(_u8L("Overflow"));
		if (win32_seh_catched == STATUS_FLOAT_UNDERFLOW)
			throw Slic3r::HardCrash(_u8L("Underflow"));
#ifdef STATUS_FLOATING_RESEVERED_OPERAND
		if (win32_seh_catched == STATUS_FLOATING_RESEVERED_OPERAND)
			throw Slic3r::HardCrash(_u8L("Floating reserved operand"));
#endif // STATUS_FLOATING_RESEVERED_OPERAND
		if (win32_seh_catched == STATUS_STACK_OVERFLOW)
			throw Slic3r::HardCrash(_u8L("Stack overflow"));
	}
}

// Wrapper for Win32 structured exceptions. Win32 structured exception blocks and C++ exception blocks cannot be mixed in the same function.
unsigned long BackgroundSlicingProcess::call_process_seh(std::exception_ptr &ex) throw()
{
	unsigned long win32_seh_catched = 0;
	__try {
		this->call_process(ex);
	} __except (is_win32_seh_harware_exception(GetExceptionCode())) {
		win32_seh_catched = GetExceptionCode();
	}
	return win32_seh_catched;
}
void BackgroundSlicingProcess::call_process_seh_throw(std::exception_ptr &ex) throw()
{
	unsigned long win32_seh_catched = this->call_process_seh(ex);
	if (win32_seh_catched) {
		// Rethrow SEH exception as Slicer::HardCrash.
	try {
			rethrow_seh_exception(win32_seh_catched);
		} catch (...) {
			ex = std::current_exception();
		}
	}
}

class StackWalker2 : public StackWalker {
public:
    StackWalker2(ExceptType extype) : StackWalker(extype) {}
    std::string output;
    void OnOutput(LPCSTR szText) override {
        output += szText; //this will collect stack info in output string
    }
};

#endif // _WIN32

void BackgroundSlicingProcess::call_process(std::exception_ptr &ex) throw()
{
	try {
		assert(m_print != nullptr);
		switch (m_print->technology()) {
		case ptFFF: this->process_fff(); break;
		case ptSLA: this->process_sla(); break;
		default: m_print->process(); break;
		}
	} catch (CanceledException& /* ex */) {
		// Canceled, this is all right.
		assert(m_print->canceled());
		ex = std::current_exception();
	} catch (...) {
#ifdef _WIN32
		StackWalker2 sw(StackWalker::AfterCatch);
		sw.ShowCallstack();
		BOOST_LOG_TRIVIAL(error) << "EXCEPTION during the print->process():" << sw.output;
		// also wrote it on disk
		try {
			std::ofstream myfile;
			myfile.open("crash_log.txt", std::ofstream::out | std::ofstream::app);
			myfile << "\nEXCEPTION during the print->process():\n";
			myfile << sw.output << "\n";
			myfile.close();
		} catch (...) {}
#endif // _WIN32
		ex = std::current_exception();
	}
}

#ifdef _WIN32
unsigned long BackgroundSlicingProcess::thread_proc_safe_seh() throw()
{
	unsigned long win32_seh_catched = 0;
	__try {
		this->thread_proc_safe();
	} __except (is_win32_seh_harware_exception(GetExceptionCode())) {
		win32_seh_catched = GetExceptionCode();
	}
	return win32_seh_catched;
}
void BackgroundSlicingProcess::thread_proc_safe_seh_throw() throw()
{
	unsigned long win32_seh_catched = this->thread_proc_safe_seh();
	if (win32_seh_catched) {
		// Rethrow SEH exception as Slicer::HardCrash.
		try {
			rethrow_seh_exception(win32_seh_catched);
		} catch (...) {
			wxTheApp->OnUnhandledException();
		}
	}
}
#endif // _WIN32

void BackgroundSlicingProcess::thread_proc_safe() throw()
{
	try {
		this->thread_proc();
	} catch (...) {
		wxTheApp->OnUnhandledException();
   	}
}

void BackgroundSlicingProcess::join_background_thread()
{
	std::unique_lock<std::mutex> lck(m_mutex);
	if (m_state == STATE_INITIAL) {
		// Worker thread has not been started yet.
		assert(! m_thread.joinable());
	} else {
		assert(m_state == STATE_IDLE);
		assert(m_thread.joinable());
		// Notify the worker thread to exit.
		m_state = STATE_EXIT;
		lck.unlock();
		m_condition.notify_one();
		// Wait until the worker thread exits.
		m_thread.join();
	}
}

bool BackgroundSlicingProcess::start()
{
	if (m_print->empty())
		// The print is empty (no object in Model, or all objects are out of the print bed).
		return false;

	// Read the preference on the UI thread, then apply it only while the worker is
	// idle under m_mutex. The core pipeline stores the policy on Print so it never
	// accesses GUI configuration from the background thread.
	const bool has_ordering_checkpoint_preference =
		m_print == m_fff_print && m_fff_print != nullptr &&
		GUI::wxGetApp().app_config != nullptr;
	const bool keep_ordering_checkpoint = has_ordering_checkpoint_preference &&
		GUI::wxGetApp().app_config->get_bool("keep_ordering_checkpoint");

	std::unique_lock<std::mutex> lck(m_mutex);
	if (m_state == STATE_INITIAL) {
		// The worker thread is not running yet. Start it.
		assert(! m_thread.joinable());
		m_thread = create_thread([this]{
#ifdef _WIN32
			this->thread_proc_safe_seh_throw();
#else // _WIN32
			this->thread_proc_safe();
#endif // _WIN32
		});
		// Wait until the worker thread is ready to execute the background processing task.
		m_condition.wait(lck, [this](){ return m_state == STATE_IDLE; });
	}
	assert(m_state == STATE_IDLE || this->running());
	if (this->running())
		// The background processing thread is already running.
		return false;
	if (! this->idle())
		throw Slic3r::RuntimeError("Cannot start a background task, the worker thread is not idle.");
	if (has_ordering_checkpoint_preference)
		m_fff_print->set_keep_ordering_checkpoint(keep_ordering_checkpoint);
	m_state = STATE_STARTED;
	m_print->set_cancel_callback([this](){ this->stop_internal(); });
	lck.unlock();
	m_condition.notify_one();
	return true;
}

// To be called on the UI thread.
bool BackgroundSlicingProcess::stop()
{
	// m_print->state_mutex() shall NOT be held. Unfortunately there is no interface to test for it.
	std::unique_lock<std::mutex> lck(m_mutex);
	if (m_state == STATE_INITIAL) {
//		m_export_path.clear();
		return false;
	}
//	assert(this->running());
	if (m_state == STATE_STARTED || m_state == STATE_RUNNING) {
		// Cancel any task planned by the background thread on UI thread.
		cancel_ui_task(m_ui_task);
		m_print->cancel();
		// Wait until the background processing stops by being canceled.
		m_condition.wait(lck, [this](){ return m_state == STATE_CANCELED; });
		// In the "Canceled" state. Reset the state to "Idle".
		m_state = STATE_IDLE;
		m_print->set_cancel_callback([](){});
	} else if (m_state == STATE_FINISHED || m_state == STATE_CANCELED) {
		// In the "Finished" or "Canceled" state. Reset the state to "Idle".
		m_state = STATE_IDLE;
		m_print->set_cancel_callback([](){});
	}
//	m_export_path.clear();
	return true;
}

bool BackgroundSlicingProcess::reset()
{
	bool stopped = this->stop();
	this->reset_export();
	m_print->clear();
	this->invalidate_all_steps();
	return stopped;
}

// To be called by Print::apply() on the UI thread through the Print::m_cancel_callback to stop the background
// processing before changing any data of running or finalized milestones.
// This function shall not trigger any UI update through the wxWidgets event.
void BackgroundSlicingProcess::stop_internal()
{
	// m_print->state_mutex() shall be held. Unfortunately there is no interface to test for it.
	if (m_state == STATE_IDLE)
		// The worker thread is waiting on m_mutex/m_condition for wake up. The following lock of the mutex would block.
		return;
	std::unique_lock<std::mutex> lck(m_mutex);
	assert(m_state == STATE_STARTED || m_state == STATE_RUNNING || m_state == STATE_FINISHED || m_state == STATE_CANCELED);
	if (m_state == STATE_STARTED || m_state == STATE_RUNNING) {
		// Cancel any task planned by the background thread on UI thread.
		cancel_ui_task(m_ui_task);
		// At this point of time the worker thread may be blocking on m_print->state_mutex().
		// Set the print state to canceled before unlocking the state_mutex(), so when the worker thread wakes up,
		// it throws the CanceledException().
		m_print->cancel_internal();
		// Allow the worker thread to wake up if blocking on a milestone.
		m_print->state_mutex().unlock();
		// Wait until the background processing stops by being canceled.
		m_condition.wait(lck, [this](){ return m_state == STATE_CANCELED; });
		// Lock it back to be in a consistent state.
		m_print->state_mutex().lock();
	}
	// In the "Canceled" state. Reset the state to "Idle".
	m_state = STATE_IDLE;
	m_print->set_cancel_callback([](){});
}

// Execute task from background thread on the UI thread. Returns true if processed, false if cancelled. 
bool BackgroundSlicingProcess::execute_ui_task(std::function<void()> task)
{
	bool running = false;
	if (m_mutex.try_lock()) {
		// Cancellation is either not in process, or already canceled and waiting for us to finish.
		// There must be no UI task planned.
		assert(! m_ui_task);
		if (! m_print->canceled()) {
			running = true;
			m_ui_task = std::make_shared<UITask>();
		}
		m_mutex.unlock();
	} else {
		// Cancellation is in process.
	}

	bool result = false;
	if (running) {
		std::shared_ptr<UITask> ctx = m_ui_task;
		GUI::wxGetApp().mainframe->m_plater->CallAfter([task, ctx]() {
			// Running on the UI thread, thus ctx->state does not need to be guarded with mutex against ::cancel_ui_task().
			assert(ctx->state == UITask::Planned || ctx->state == UITask::Canceled);
			if (ctx->state == UITask::Planned) {
				task();
				std::unique_lock<std::mutex> lck(ctx->mutex);
	    		ctx->state = UITask::Finished;
	    	}
	    	// Wake up the worker thread from the UI thread.
    		ctx->condition.notify_all();
	    });

	    {
			std::unique_lock<std::mutex> lock(ctx->mutex);
	    	ctx->condition.wait(lock, [&ctx]{ return ctx->state == UITask::Finished || ctx->state == UITask::Canceled; });
	    }
	    result = ctx->state == UITask::Finished;
		m_ui_task.reset();
	}

	return result;
}

// Under m_mutex on the UI thread, mark the shared task cancelled, wake its worker, and invoke requester cancellation without touching windows; forms queue cancellation and native pickers observe it after returning.
void BackgroundSlicingProcess::cancel_ui_task(std::shared_ptr<UITask> task)
{
	assert(wxIsMainThread());
	if (task) {
		std::unique_lock<std::mutex> lck(task->mutex);
		task->state = UITask::Canceled;
		lck.unlock();
		task->condition.notify_all();
		// The task may be inside the form or native picker; the callback requests cancellation without touching windows while the callers hold locks.
		if (m_close_inputs_dialog)
			m_close_inputs_dialog();
	}
}

// On the UI thread outside locks, request cancellation and move an accepted continuation into process state until return; application then forced closing takes precedence, and false means none is open.
bool BackgroundSlicingProcess::close_inputs_dialog(std::function<void()> after, bool forced, bool closing)
{
	if (!m_close_inputs_dialog)
		return false;
	// A closing of the application wins over another kind of continuation (the recreation of the GUI); between closings a forced
	// one wins; otherwise the first request is kept.
	const bool replace = !m_after_inputs_dialog || (closing && !m_after_inputs_closing) ||
		(closing && forced && !m_after_inputs_forced);
	if (after && replace) {
		m_after_inputs_dialog = std::move(after);
		m_after_inputs_closing = closing;
		m_after_inputs_forced = forced;
	}
	m_close_inputs_dialog();
	return true;
}

// On the UI thread, clear the requester's closer, move its continuation to the event queue after task return, and clear continuation flags.
void BackgroundSlicingProcess::end_inputs_dialog()
{
	m_close_inputs_dialog = nullptr;
	if (m_after_inputs_dialog) {
		GUI::wxGetApp().CallAfter(std::move(m_after_inputs_dialog));
		m_after_inputs_dialog = nullptr;
		m_after_inputs_closing = false;
		m_after_inputs_forced = false;
	}
}

bool BackgroundSlicingProcess::empty() const
{
	assert(m_print != nullptr);
	return m_print->empty();
}

std::pair<PrintBase::PrintValidationError, std::string> BackgroundSlicingProcess::validate(std::vector<std::string>* warnings)
{
	assert(m_print != nullptr);
    return m_print->validate(warnings);
}

// Apply config over the print. Returns false, if the new config values caused any of the already
// processed steps to be invalidated, therefore the task will need to be restarted.
Print::ApplyStatus BackgroundSlicingProcess::apply(const Model &model, const DynamicPrintConfig &config, const DynamicPrintConfig *physical_printer_config)
{
	assert(m_print != nullptr);
	assert(config.opt_enum<PrinterTechnology>("printer_technology") == m_print->technology());
	Print::ApplyStatus invalidated = m_print->apply(model, config);
    if (physical_printer_config && 
        (m_print->physical_printer_config().diff(*physical_printer_config).size() != 0 || m_print->physical_printer_config().keys() != physical_printer_config->keys()) ) {
        // Export may read these options; stop the worker before replacing their storage.
        std::scoped_lock<std::mutex> lock(m_print->state_mutex());
        if (m_print->technology() == ptFFF)
            m_fff_print->request_replay_from(Steps::Checkpoint::Plan);
        else
            m_print->call_cancel_callback();
        m_print->physical_printer_config().clear();
        m_print->physical_printer_config().apply(*physical_printer_config);
        if (!invalidated)
            invalidated = PrintBase::APPLY_STATUS_INVALIDATED;
    }
	if ((invalidated & PrintBase::APPLY_STATUS_INVALIDATED) != 0 && m_print->technology() == ptFFF &&
		!m_fff_print->is_step_done(STEP_GCODE)) {
		// Some FFF status was invalidated, and the G-code was not exported yet.
		// Let the G-code preview UI know that the final G-code preview is not valid.
		// In addition, this early memory deallocation reduces memory footprint.
		if (m_gcode_result != nullptr)
			m_gcode_result->reset();
	}
	return invalidated;
}

void BackgroundSlicingProcess::set_task(const PrintBase::TaskParams &params)
{
	assert(m_print != nullptr);
	m_print->set_task(params);
}

// Copy the supplied SLA destination and invalidate finalization under the print state lock; leave pending work unchanged when an export or upload is already queued.
void BackgroundSlicingProcess::schedule_export(const std::string &path)
{
	assert(!this->is_export_scheduled() && m_upload_job.empty());
	if (this->is_export_scheduled() || !m_upload_job.empty())
		return;

	std::scoped_lock<std::mutex> lock(m_print->state_mutex());
	this->invalidate_step(bspsGCodeFinalize);
	m_export_path = path;
}

// Copy the FFF suggestion directory and base, store the ASCII-fallback flag, and invalidate finalization under the print state lock; leave pending work unchanged when another export or upload is queued.
void BackgroundSlicingProcess::schedule_fff_export(const std::string &suggestion_directory,
                                                   const std::string &suggestion_filename_base,
                                                   bool ascii_fallback)
{
	assert(!this->is_export_scheduled() && m_upload_job.empty());
	if (this->is_export_scheduled() || !m_upload_job.empty())
		return;

	std::scoped_lock<std::mutex> lock(m_print->state_mutex());
	this->invalidate_step(bspsGCodeFinalize);
	m_fff_export_suggestion_directory = suggestion_directory;
	m_fff_export_suggestion_filename_base = suggestion_filename_base;
	m_fff_export_suggestion_ascii_fallback = ascii_fallback;
	m_fff_export_scheduled = true;
}

// Move the upload job into the pending request and invalidate finalization under the print state lock; leave pending work unchanged when an export or upload is already queued.
void BackgroundSlicingProcess::schedule_upload(Slic3r::PrintHostJob upload_job)
{
	assert(!this->is_export_scheduled() && m_upload_job.empty());
	if (this->is_export_scheduled() || !m_upload_job.empty())
		return;

	std::scoped_lock<std::mutex> lock(m_print->state_mutex());
	this->invalidate_step(bspsGCodeFinalize);
	m_upload_job = std::move(upload_job);
}

// When the worker is idle, clear the SLA destination and FFF suggestions and invalidate finalization under the print state lock; leave them unchanged while the worker is running.
void BackgroundSlicingProcess::reset_export()
{
	bool running = true;
	{
		// I don't know if it's safe to let m_mutex be lock while doing invalidate_step.
		// if so, please remove the braces.
		std::unique_lock<std::mutex> lck(m_mutex);
		running = this->running();
		//assert(!running);
	}
	if (!running) {
		m_export_path.clear();
		m_fff_export_scheduled = false;
		m_fff_export_suggestion_directory.clear();
		m_fff_export_suggestion_filename_base.clear();
		m_fff_export_suggestion_ascii_fallback = false;
		// invalidate_step expects the mutex to be locked.
		std::scoped_lock<std::mutex> lock(m_print->state_mutex());
		this->invalidate_step(bspsGCodeFinalize);
	}
}

bool BackgroundSlicingProcess::set_step_started(BackgroundSlicingProcessStep step)
{ 
	return m_step_state.set_started(step, m_print->state_mutex(), [this](){ this->throw_if_canceled(); });
}

void BackgroundSlicingProcess::set_step_done(BackgroundSlicingProcessStep step)
{ 
	m_step_state.set_done(step, m_print->state_mutex(), [this](){ this->throw_if_canceled(); });
}

bool BackgroundSlicingProcess::is_step_done(BackgroundSlicingProcessStep step) const
{ 
	return m_step_state.is_done(step, m_print->state_mutex());
}

bool BackgroundSlicingProcess::invalidate_step(BackgroundSlicingProcessStep step)
{
    bool invalidated = m_step_state.invalidate(step, [this](){ this->stop_internal(); });
    return invalidated;
}

bool BackgroundSlicingProcess::invalidate_all_steps()
{ 
	return m_step_state.invalidate_all([this](){ this->stop_internal(); });
}

// Build a single-use STEP_EXPORT request from the FFF Print's configuration and a renderer borrowing this process; attach the GUI input callback and path suggestions, deposit it on the Print, and propagate export failures.
void BackgroundSlicingProcess::finalize_gcode()
{
	Print::ExportRequest request = m_fff_print->make_export_request(
		[this](const ThumbnailsParams &params) { return this->render_thumbnails(params); });
	request.inputs = [this](ExportInputForm &form) { return this->request_export_inputs(form); };
	request.output_path_suggestion_directory = m_fff_export_suggestion_directory;
	request.output_path_suggestion_filename_base = m_fff_export_suggestion_filename_base;
	request.output_path_suggestion_ascii_fallback = m_fff_export_suggestion_ascii_fallback;
	m_fff_print->request_export(std::move(request));
	Orchestrator::instance().export_requested(*m_fff_print);
	m_print->set_status(100, _u8L("G-code export finished."));
}

// Export the generated reference with the profile's exporter (thumbnails rendered through render_thumbnails) into a private upload file
// and queue the upload; the file is removed if the upload name cannot be resolved. The post-processing scripts have already modified
// the reference in STEP_GCODE. SLA writes its raster file instead. The borrowed Print supplies QUERY services; failures propagate before queue ownership is transferred.
void BackgroundSlicingProcess::prepare_upload()
{
	// Generate a unique temp path to which the gcode/zip file is copied/exported
	boost::filesystem::path source_path = boost::filesystem::temp_directory_path()
		/ boost::filesystem::unique_path("." SLIC3R_APP_KEY ".upload.%%%%-%%%%-%%%%-%%%%");

	// Guards the private export file until the upload queue has accepted it.
	std::optional<ScopedTemporaryFile> private_file;

	if (m_print == m_fff_print) {
		// The request holds the configuration copy (taken under the print state mutex) and the thumbnail renderer.
		Print::ExportRequest request = m_fff_print->make_export_request(
			[this](const ThumbnailsParams &params) { return this->render_thumbnails(params); });
		request.inputs = [this](ExportInputForm &form) { return this->request_export_inputs(form); };
		const Orchestrator::GCodeExporterSelection selection =
			Orchestrator::instance().select_gcode_exporter(*m_fff_print, request.config);
		// The temporary path must end with the exporter extension, otherwise the random part would be taken for a suffix.
		const std::string private_path = selection.temporary_path("." SLIC3R_APP_KEY ".upload.");
		private_file.emplace(private_path);
		request.set_output_path(private_path);
		m_fff_print->request_export(std::move(request));
		Orchestrator::instance().export_requested(*m_fff_print);
		const std::string exported_path = m_fff_print->export_result().local_path;
		if (exported_path.empty())
			throw Slic3r::ExportError(_u8L("The selected G-code exporter did not write a local file."));
		source_path = boost::filesystem::path(exported_path);
		if (m_upload_job.upload_data.upload_path.extension().empty())
			m_upload_job.upload_data.upload_path += selection.extension;
		m_upload_job.upload_data.upload_path = boost::filesystem::path(
			m_fff_print->print_statistics().finalize_output_path(
				m_upload_job.upload_data.upload_path.string()));
    } else if (m_print == m_sla_print) {
		m_upload_job.upload_data.upload_path = m_sla_print->print_statistics().finalize_output_path(m_upload_job.upload_data.upload_path.string());
        Vec2ds sizes = current_print()->full_print_config().option<ConfigOptionPoints>("thumbnails")->get_values();
        ThumbnailsParams params{sizes, true, true, true, true};
        apply_thumbnail_colors(params, current_print()->full_print_config());
        ThumbnailsList thumbnails = this->render_thumbnails(params);
        m_sla_print->export_print(source_path.string(),thumbnails, m_upload_job.upload_data.upload_path.filename().string());
    }

    m_print->set_status(100, GUI::format(_L("Scheduling upload to `%1%`. See Window -> Print Host Upload Queue"), m_upload_job.printhost->get_host()));

	m_upload_job.upload_data.source_path = std::move(source_path);

	GUI::wxGetApp().printhost_job_queue().enqueue(std::move(m_upload_job));
	// The queue accepted the job: it owns the file now. Any earlier failure removes the private file instead.
	if (private_file)
		private_file->release();
}

// Executed by the background thread, to start a task on the UI thread.
ThumbnailsList BackgroundSlicingProcess::render_thumbnails(const ThumbnailsParams &params)
{
	ThumbnailsList thumbnails;
	if (m_thumbnail_cb)
		this->execute_ui_task([this, &params, &thumbnails](){ thumbnails = m_thumbnail_cb(params); });
	return thumbnails;
}

namespace {
// Data shared by the export-input requester and its waiting worker. A cancelled worker can return while the UI task is still in a
// form or native picker, so the task retains this exchange until it finishes and the worker reads it only after successful completion.
struct ExportInputsExchange
{
	// Copy of the form the requester works on; on validation it holds the answers.
	ExportInputForm form;
	// Whether the user validated the requester.
	bool            validated = false;
	// Whether stop or application closing asked the native picker to cancel after it returns.
	bool            cancellation_requested = false;
	// Message of the exception that stopped the requester, empty when there was none.
	std::string     error;
};
} // namespace

// Copy the caller's form into shared UI-task storage, use the native picker for is_single_path_input() or ExportInputsDialog otherwise, and move answers back only after validation; a cancelled worker leaves the exchange to the UI task, whose exceptions are rethrown here.
bool BackgroundSlicingProcess::request_export_inputs(ExportInputForm &form)
{
	std::shared_ptr<ExportInputsExchange> exchange = std::make_shared<ExportInputsExchange>();
	exchange->form = form;
	const bool completed = this->execute_ui_task([this, exchange]() {
		try {
			if (is_single_path_input(exchange->form.definitions)) {
				m_close_inputs_dialog = [exchange]() { exchange->cancellation_requested = true; };
				const ConfigOptionDef &definition = exchange->form.definitions.front();
				ConfigOptionString *path_option = exchange->form.values.option<ConfigOptionString>(definition.opt_key);
				while (!exchange->cancellation_requested) {
					if (!GUI::PathCtrl::choose_path(GUI::wxGetApp().mainframe, definition, path_option->value, false, {}))
						break;
					if (exchange->cancellation_requested)
						break;
					if (definition.gui_type == ConfigOptionDef::GUIType::save_path) {
						const boost::filesystem::path selected_path = GUI::into_path(GUI::from_u8(path_option->value));
						if (!GUI::validate_export_save_filename(GUI::wxGetApp().mainframe, selected_path.filename().string())) {
							if (exchange->cancellation_requested)
								break;
							continue;
						}
					}
					exchange->validated = true;
					break;
				}
			} else {
				GUI::ExportInputsDialog dialog(GUI::wxGetApp().mainframe, exchange->form);
				wxWeakRef<GUI::ExportInputsDialog> weak_dialog(&dialog);
				m_close_inputs_dialog = [weak_dialog]() {
					GUI::wxGetApp().CallAfter([weak_dialog]() {
						if (GUI::ExportInputsDialog *open_dialog = weak_dialog.get())
							open_dialog->request_cancel();
					});
				};
				const int result = dialog.ShowModal();
				exchange->validated = result == wxID_OK;
				exchange->error     = dialog.error();
			}
		} catch (const std::exception &ex) {
			exchange->error = ex.what();
		} catch (...) {
			exchange->error = "Unknown error while asking for the inputs of the export.";
		}
		this->end_inputs_dialog();
	});
	if (!completed)
		return false;
	if (!exchange->error.empty())
		throw Slic3r::RuntimeError(exchange->error);
	if (!exchange->validated)
		return false;
	form.values = std::move(exchange->form.values);
	return true;
}

}; // namespace Slic3r
