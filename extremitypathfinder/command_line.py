import argparse

from extremitypathfinder import PolygonEnvironment
from extremitypathfinder.plotting import PlottingEnvironment
from extremitypathfinder.helper_fcts import read_json

environment = PlottingEnvironment()

def main():
    parser = argparse.ArgumentParser(description="description of parser")
    parser.add_argument("json_file", type=str, help="file to be read")
    parser.add_argument("start", type=str, help="start coordinates for search")
    parser.add_argument("goal", type=str, help="goal coordinates for search")

    # Takes input from sys.argv
    parsed_args = parser.parse_args()

    # Parse JSON file
    list_of_boundaries, list_of_holes = read_json(parsed_args.json_file)

    # Parse tuples from strings
    start_coordinates = eval(parsed_args.start)
    goal_coordinates = eval(parsed_args.goal)

    # Execute search for the given parameters
    environment.store(list_of_boundaries, list_of_holes, validate=False)
    environment.prepare()
    path, distance = environment.find_shortest_path(start_coordinates, goal_coordinates)

