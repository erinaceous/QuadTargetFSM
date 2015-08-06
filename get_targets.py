#!/usr/bin/env python2
# vim: set tabstop=4 shiftwidth=4 textwidth=79 cc=72,79:
"""
    get_targets: Run the quadtarget program and parse the JSON data
    it outputs (example of how to use the program from python code)
    Original Author: Owain Jones [github.com/erinaceous] [contact@odj.me]
"""

from __future__ import print_function
import asyncproc
import argparse
import atexit
try:
    import simplejson as json
    JSONError = json.JSONDecodeError
except ImportError:
    try:
        import cjson as json
        json.loads = json.decode
        JSONError = json.DecodeError
    except ImportError:
        import json
        JSONError = ValueError
import time
import os


p = None


def kill_proc():
    p.kill()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-o', '--output-video', default=None)
    return parser.parse_args()


if __name__ == '__main__':
    atexit.register(kill_proc)
    args = parse_args()
    print()
    p = asyncproc.Process('./QuadTarget')
    while True:
        poll = p.wait(os.WNOHANG)
        if poll is not None:
            break
        out = p.read()
        if out != "":
            try:
                decoded = json.loads(out)
            except JSONError as e:
                print(e)
                print(out)
                continue
            if 'fps' in decoded:
                print('fps:', decoded['fps'], end=' ')
            if 'targets' in decoded:
                print('targets:', len(decoded['targets']), end='')
            else:
                print('targets: 0     ', end='')
            print('', end='\r')
            time.sleep(0)
    print()
