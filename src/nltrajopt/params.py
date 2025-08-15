import sys
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--vis", action="store_true")
args = parser.parse_args()

VIS = args.vis
MAX_CPU_TIME = 600.0

SPACE_NQ = 6

def get_repr_nq(model):
    return model.nv
