import argparse

from extremitypathfinder.helper_classes import DirectedHeuristicGraph, Edge, Polygon, PolygonVertex, Vertex
from extremitypathfinder.helper_fcts import (
    check_data_requirements, convert_gridworld, find_visible, find_within_range, inside_polygon, read_json)


def main():
    parser = argparse.ArgumentParser(description="description of parser")
    parser.add_argument("json_file", type=str, help="file to be read")
    parser.add_argument("start", type=float, help="start coordinates for search")
    parser.add_argument("goal", type=float, help="goal coordinates for search")

    args = parser.parse_args()
    print(args)
    print(read_json("file"))
