"""This package supplies a plugin with an invalid alternative preference to loader tests.

A plugin is a component loaded by the slicer application. An exclusive group
contains alternatives for one processing stage; group priority ranks those
alternatives and must fit a signed 32-bit integer.

The Python loader calls register_plugin(api) and reads the returned PluginBase
object's preference before registration. Tests inspect the resulting package error.

Call graph:
    register_plugin
        PluginBase
"""
from slic3r_api import PluginBase, STEP_POST_SLICING


def register_plugin(api):
    """Return a disabled plugin with preference 2**31, which the loader rejects before registration."""
    return PluginBase("python.test.priority.high", STEP_POST_SLICING, exclusive_group="",
                      exclusive_group_priority=2**31)
