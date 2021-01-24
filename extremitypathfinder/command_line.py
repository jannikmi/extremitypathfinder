import argparse

from extremitypathfinder import PolygonEnvironment
from extremitypathfinder.plotting import PlottingEnvironment
from extremitypathfinder.helper_fcts import read_json

environment = PlottingEnvironment()

def main():
    parser = argparse.ArgumentParser(description="")
    parser.add_argument("json_file", type=str, help="JSON file to be read")
    parser.add_argument("start", type=str, help="string containinng start coordinates as a tuple: '(2.5, 3.2)'")
    parser.add_argument("goal", type=str, help="string containinng start coordinates as a tuple: '(7.9, 6.8)'")
    # parser.add_argument("-h", action='store_true', help="help flag")

    # Takes input from sys.argv
    parsed_args = parser.parse_args()

    # if(parsed_args.h):
    #     print("blabla")

    # Parse JSON file
    list_of_boundaries, list_of_holes = read_json(parsed_args.json_file)

    # Parse tuples from strings
    start_coordinates = eval(parsed_args.start)
    goal_coordinates = eval(parsed_args.goal)

    # Execute search for the given parameters
    environment.store(list_of_boundaries, list_of_holes, validate=False)
    environment.prepare()
    path, distance = environment.find_shortest_path(start_coordinates, goal_coordinates)

