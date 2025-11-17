#!/usr/bin/env python3
"""Simple smoke test for build_updates.py
Runs the build script and verifies that site/index.html exists and contains the UPDATES marker.
"""
import subprocess
import os
import sys

ROOT = os.path.dirname(os.path.dirname(__file__))
SITE_INDEX = os.path.join(ROOT, 'site', 'index.html')

ret = subprocess.call([sys.executable, os.path.join(ROOT, 'scripts', 'build_updates.py')])
if ret != 0:
    print('build_updates.py exited with', ret)
    sys.exit(ret)

if not os.path.exists(SITE_INDEX):
    print('site/index.html not found')
    sys.exit(2)
with open(SITE_INDEX, 'r', encoding='utf8') as fh:
    data = fh.read()
if '<!-- UPDATES-START -->' not in data or '<!-- UPDATES-END -->' not in data:
    print('Markers not found in generated site/index.html')
    sys.exit(3)
print('Smoke test passed: site/index.html created and contains markers')
