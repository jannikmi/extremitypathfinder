.. list-table:: Query latency
   :header-rows: 1

   * - Vertices
     - Package
     - p50
     - p95
     - p99
     - Mean
     - Queries/s
     - Queries
   * - 251
     - extremitypathfinder
     - 594 µs
     - 704 µs
     - 749 µs
     - 539 µs
     - 1.86e+03
     - 200 / 200
   * - 251
     - extremitypathfinder without Numba
     - 961 µs
     - 1.08 ms
     - 1.15 ms
     - 860 µs
     - 1.16e+03
     - 200 / 200
   * - 251
     - pyvisgraph
     - 9.41 ms
     - 11.4 ms
     - 11.6 ms
     - 9.4 ms
     - 106
     - 200 / 200
   * - 508
     - extremitypathfinder
     - 1.02 ms
     - 1.39 ms
     - 4.87 ms
     - 1.05 ms
     - 948
     - 200 / 200
   * - 508
     - extremitypathfinder without Numba
     - 1.79 ms
     - 2.1 ms
     - 2.26 ms
     - 1.63 ms
     - 614
     - 200 / 200
   * - 508
     - pyvisgraph
     - 20.8 ms
     - 26.2 ms
     - 27.1 ms
     - 21 ms
     - 47.5
     - 200 / 200
   * - 1,009
     - extremitypathfinder
     - 1.84 ms
     - 2.52 ms
     - 3.04 ms
     - 1.77 ms
     - 565
     - 200 / 200
   * - 1,009
     - extremitypathfinder without Numba
     - 3.42 ms
     - 4.36 ms
     - 5.25 ms
     - 3.2 ms
     - 313
     - 200 / 200
   * - 1,009
     - pyvisgraph
     - 48 ms
     - 63.7 ms
     - 65.6 ms
     - 48.7 ms
     - 20.5
     - 200 / 200
   * - 2,009
     - extremitypathfinder
     - 3.22 ms
     - 4.61 ms
     - 5.66 ms
     - 3.11 ms
     - 322
     - 200 / 200
   * - 2,009
     - extremitypathfinder without Numba
     - 6.68 ms
     - 8.43 ms
     - 9.39 ms
     - 6.18 ms
     - 162
     - 200 / 200
   * - 2,009
     - pyvisgraph
     - 110 ms
     - 148 ms
     - 153 ms
     - 109 ms
     - 9.18
     - 200 / 200
   * - 3,046
     - extremitypathfinder
     - 4.5 ms
     - 6.42 ms
     - 7.41 ms
     - 4.31 ms
     - 232
     - 200 / 200
   * - 3,046
     - extremitypathfinder without Numba
     - 8.73 ms
     - 10.7 ms
     - 12 ms
     - 8.02 ms
     - 125
     - 200 / 200
   * - 4,018
     - extremitypathfinder
     - 5.64 ms
     - 7.85 ms
     - 9.18 ms
     - 5.35 ms
     - 187
     - 200 / 200
   * - 4,018
     - extremitypathfinder without Numba
     - 10.9 ms
     - 14 ms
     - 15.1 ms
     - 10.1 ms
     - 99
     - 200 / 200
   * - 5,020
     - extremitypathfinder
     - 8.44 ms
     - 14.2 ms
     - 18 ms
     - 8.48 ms
     - 118
     - 200 / 200
   * - 5,020
     - extremitypathfinder without Numba
     - 18.5 ms
     - 24.4 ms
     - 28.7 ms
     - 17.4 ms
     - 57.6
     - 200 / 200
   * - 6,097
     - extremitypathfinder
     - 10.5 ms
     - 18.7 ms
     - 23.7 ms
     - 10.8 ms
     - 92.8
     - 200 / 200
   * - 6,097
     - extremitypathfinder without Numba
     - 23 ms
     - 31.6 ms
     - 36.8 ms
     - 21.9 ms
     - 45.7
     - 200 / 200
