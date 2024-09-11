#!/usr/bin/env python3

from __future__ import print_function
import jinja2
import argparse
import os
import math
import numpy as np

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('filename')
    parser.add_argument('env_dir')
    args = parser.parse_args()
    env = jinja2.Environment(loader=jinja2.FileSystemLoader(args.env_dir))
    template = env.get_template(os.path.relpath(args.filename, args.env_dir))

    try:
        import rospkg
        rospack = rospkg.RosPack()
    except ImportError:
        pass
        rospack = None

    d = {'np': np, 'rospack': rospack, 'math': math}
    result = template.render(d)
    filename_out = args.filename.replace('.sdf.jinja','.sdf')
    with open(filename_out, 'w') as f_out:
        print('{:s} -> {:s}'.format(args.filename, filename_out))
        f_out.write(result)