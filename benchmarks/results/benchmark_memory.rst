.. list-table:: Memory above the baseline: the peak of the build and the queries, and what is retained after the build. The baseline (imports and warm-up) and the total peak
   :header-rows: 1

   * - Vertices
     - Package
     - Peak
     - Retained
     - Baseline
     - Total peak
   * - 249
     - extremitypathfinder
     - < 1 MiB
     - < 1 MiB
     - 88.6 MiB
     - 89.5 MiB
   * - 249
     - extremitypathfinder without Numba
     - 1.45 MiB
     - < 1 MiB
     - 37.5 MiB
     - 38.9 MiB
   * - 249
     - pyvisgraph
     - < 1 MiB
     - < 1 MiB
     - 18.9 MiB
     - 19.7 MiB
   * - 494
     - extremitypathfinder
     - 1.88 MiB
     - 1.33 MiB
     - 88.6 MiB
     - 90.5 MiB
   * - 494
     - extremitypathfinder without Numba
     - 2.28 MiB
     - 1.33 MiB
     - 37.5 MiB
     - 39.8 MiB
   * - 494
     - pyvisgraph
     - 2 MiB
     - 1.66 MiB
     - 19.1 MiB
     - 21.1 MiB
   * - 994
     - extremitypathfinder
     - 5.16 MiB
     - 2.94 MiB
     - 88.8 MiB
     - 94 MiB
   * - 994
     - extremitypathfinder without Numba
     - 7.45 MiB
     - 2.94 MiB
     - 37.5 MiB
     - 44.9 MiB
   * - 994
     - pyvisgraph
     - 3.92 MiB
     - 3.52 MiB
     - 19.2 MiB
     - 23.2 MiB
   * - 1,985
     - extremitypathfinder
     - 12 MiB
     - 5.83 MiB
     - 89.1 MiB
     - 101 MiB
   * - 1,985
     - extremitypathfinder without Numba
     - 13.4 MiB
     - 5.83 MiB
     - 37.8 MiB
     - 51.1 MiB
   * - 1,985
     - pyvisgraph
     - 11.6 MiB
     - 10.5 MiB
     - 19.3 MiB
     - 30.9 MiB
   * - 4,954
     - extremitypathfinder
     - 19 MiB
     - 13.2 MiB
     - 90.1 MiB
     - 109 MiB
   * - 4,954
     - extremitypathfinder without Numba
     - 17.3 MiB
     - 13.2 MiB
     - 38.5 MiB
     - 55.8 MiB
   * - 4,954
     - pyvisgraph
     - 57 MiB
     - 55.6 MiB
     - 20.1 MiB
     - 77.1 MiB
   * - 9,952
     - extremitypathfinder
     - 62.8 MiB
     - 55.4 MiB
     - 90.6 MiB
     - 153 MiB
   * - 9,952
     - extremitypathfinder without Numba
     - 62.1 MiB
     - 55.4 MiB
     - 39.7 MiB
     - 102 MiB

Baseline without input data (after the imports and the warm-up): extremitypathfinder with Numba 88.5 MiB, extremitypathfinder without Numba 37.2 MiB, pyvisgraph 18.8 MiB.
