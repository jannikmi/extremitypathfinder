import argparse

from extremitypathfinder import PolygonEnvironment
from extremitypathfinder.configs import BOUNDARY_JSON_KEY, HOLES_JSON_KEY
from extremitypathfinder.utils import read_json

JSON_HELP_MSG = (
    "path to the JSON file to be read. "
    f'The JSON file must have 2 keys: "{BOUNDARY_JSON_KEY}" and "{HOLES_JSON_KEY}".'
    'Example: (also see "example.json")'
    '{ "boundary":[[0.0, 0.0], [10.0, 0.0],[9.0, 5.0], [10.0, 10.0], [0.0, 10.0]],'
    '"holes": [[[3.0, 7.0], [5.0, 9.0], [4.5, 7.0], [5.0, 4.0]],'
    "[[1.0, 2.0], [2.0, 2.0], [2.0, 1.0], [1.0, 1.0]]] }"
    f' The value assigned to the "{BOUNDARY_JSON_KEY}" key must be list containing '
    "a list of length 2 for each coordinate pair."
    f' The value assigned to the "{HOLES_JSON_KEY}" key must be '
    "a list of possibly multiple hole polygon lists."
)

# environment = PlottingEnvironment()
environment = PolygonEnvironment()


def main():
    parser = argparse.ArgumentParser(description="parse extremitypathfinder parameters")
    parser.add_argument("path2json_file", type=str, help=JSON_HELP_MSG)
    parser.add_argument(
        "-s",
        "--start",
        nargs=2,
        type=float,
        required=True,
        help="the start coordinates given as two float values, e.g. <2.5 3.2>",
    )
    parser.add_argument(
        "-g",
        "--goal",
        nargs=2,
        type=float,
        required=True,
        help="the goal coordinates given as two float values, e.g. <7.9 6.8>",
    )
    # TODO grid input requires different json data format
    # parser.add_argument('-g', '--grid', type=bool, help='weather the input data is a specifying a grid')
    parsed_args = parser.parse_args()  # Takes input from sys.argv

    # Parse JSON file
    boundary_coordinates, list_of_holes = read_json(parsed_args.path2json_file)

    # Parse tuples from the input arguments (interpreted as lists):
    start_coordinates = tuple(parsed_args.start)
    goal_coordinates = tuple(parsed_args.goal)

    # Execute search for the given parameters
    environment.store(boundary_coordinates, list_of_holes, validate=False)
    environment.prepare()
    path, distance = environment.find_shortest_path(start_coordinates, goal_coordinates)
    print(path, distance)
