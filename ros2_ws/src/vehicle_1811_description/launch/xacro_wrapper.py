#!/usr/bin/env python3
"""Workaround for a bug in xacro==2.1.1 (this image's installed version).

Its own source does `except xml.parsers.expat.ExpatError:` without ever
explicitly importing that submodule, which crashes with
`AttributeError: module 'xml' has no attribute 'parsers'` the moment
anything in its try block raises -- masking whatever the real error was
(or, as here, masking that there wasn't one). Pre-importing the submodule
here, before handing off to xacro's own main(), makes that attribute
lookup succeed so xacro actually runs instead of crashing in its own
exception handler.

description.launch.py uses this in place of the bare `xacro` command,
since every launch spawns a fresh subprocess with no guarantee anything
else in that process already imported xml.parsers.expat first -- running
`xacro` directly by hand can look fine in one shell and crash in the next
depending on what else happened to import it earlier in that process.

Usage: identical to the xacro CLI -- xacro_wrapper.py <args...>
"""
import sys

import xml.parsers.expat  # noqa: F401 -- imported for its side effect, see docstring above

from xacro import main

if __name__ == '__main__':
    sys.argv[0] = 'xacro'
    main()
