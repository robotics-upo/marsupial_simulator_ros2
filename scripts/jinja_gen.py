#!/usr/bin/env python3
"""
Genera un fichero .sdf a partir de una plantilla .sdf.jinja.

Uso:
  python3 jinja_gen.py <template.sdf.jinja> <env_dir> [--var key=value ...]

Ejemplos:
  # Regenerar con parámetros por defecto:
  python3 jinja_gen.py models/tether/tether.sdf.jinja models/tether

  # Sin aerodinámica (más rápido, sin viento):
  python3 jinja_gen.py models/tether/tether.sdf.jinja models/tether --var enable_drag=False

  # Cuerda más corta con menos elementos:
  python3 jinja_gen.py models/tether/tether.sdf.jinja models/tether --var number_elements=60 --var cl=0.15

Los valores de --var se evalúan como expresiones Python, por lo que:
  True/False, enteros, flotantes y strings entre comillas funcionan directamente.
"""

from __future__ import print_function
import jinja2
import argparse
import os
import math
import numpy as np


def parse_var(kv_str):
    """Parsea 'key=value' y evalúa el valor como expresión Python."""
    if '=' not in kv_str:
        raise argparse.ArgumentTypeError(f"--var requiere formato key=value, recibido: {kv_str!r}")
    key, value_str = kv_str.split('=', 1)
    try:
        value = eval(value_str)  # noqa: S307 — uso interno en scripts de investigación
    except Exception as e:
        raise argparse.ArgumentTypeError(f"No se pudo evaluar el valor {value_str!r}: {e}")
    return key, value


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Genera .sdf desde .sdf.jinja con parámetros opcionales por CLI."
    )
    parser.add_argument('filename', help="Ruta al fichero .sdf.jinja")
    parser.add_argument('env_dir', help="Directorio raíz para el loader de Jinja2")
    parser.add_argument(
        '--var', metavar='key=value', action='append', default=[],
        help="Sobreescribe una variable de la plantilla. Admite múltiples --var."
    )
    args = parser.parse_args()

    env = jinja2.Environment(loader=jinja2.FileSystemLoader(args.env_dir))
    template = env.get_template(os.path.relpath(args.filename, args.env_dir))

    try:
        import rospkg
        rospack = rospkg.RosPack()
    except ImportError:
        rospack = None

    d = {'np': np, 'rospack': rospack, 'math': math}

    # Parsear y añadir variables extra de CLI (sobrescriben los defaults de la plantilla)
    for kv in args.var:
        key, value = parse_var(kv)
        d[key] = value
        print(f'  Var override: {key} = {value!r}')

    result = template.render(d)

    filename_out = args.filename.replace('.sdf.jinja', '.sdf')
    with open(filename_out, 'w') as f_out:
        print(f'{args.filename} -> {filename_out}')
        f_out.write(result)
