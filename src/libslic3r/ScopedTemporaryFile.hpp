///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_ScopedTemporaryFile_hpp_
#define slic3r_ScopedTemporaryFile_hpp_

#include <string>
#include <utility>

#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>

/*
Typical use is to construct the guard before asking a producer to write its
private path. Keep the guard while copying or queuing that file. Call release()
only after another owner has accepted responsibility for removing the file:

    ScopedTemporaryFile temporary(path);
    // Produce and consume the file while temporary owns its cleanup.
    temporary.release(); // Only after an explicit ownership transfer.
*/

namespace Slic3r {

/*
Scoped owner of a private file path
===================================

A caller that asks a producer to write a private file (an export that will be
copied or queued afterwards) wraps the path in this guard before the request.
Whatever happens next (the producer fails, is cancelled after writing, or the
following copy fails), the destructor removes the file. Once a new owner has
taken the file for good (for example an upload queue that accepted it), the
caller calls release() and the guard does nothing.

The guard only owns the path it was given. A missing file is not an error, and
a removal failure is logged, never thrown: destructors run during unwinding.
*/
class ScopedTemporaryFile
{
public:
    explicit ScopedTemporaryFile(std::string path) : m_path(std::move(path)) {}
    ScopedTemporaryFile(const ScopedTemporaryFile &) = delete;
    ScopedTemporaryFile &operator=(const ScopedTemporaryFile &) = delete;

    ~ScopedTemporaryFile()
    {
        if (m_released)
            return;
        boost::system::error_code error;
        boost::filesystem::remove(boost::filesystem::path(m_path), error);
        if (error)
            BOOST_LOG_TRIVIAL(error) << "Failed to remove temporary file " << m_path << ": " << error.message();
    }

    // Give up the ownership: the file stays where it is.
    void release() { m_released = true; }

private:
    std::string m_path;
    bool        m_released = false;
};

} // namespace Slic3r

#endif // slic3r_ScopedTemporaryFile_hpp_
