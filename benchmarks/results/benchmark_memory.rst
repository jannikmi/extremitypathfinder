.. list-table:: Memory above the baseline: the peak of the build and the queries, and what is retained after the build. The baseline (imports and warm-up) and the total peak
   :header-rows: 1

   * - Vertices
     - Package
     - Peak
     - Retained
     - Baseline
     - Total peak
   * - 251
     - extremitypathfinder
     - < 1 MiB
     - < 1 MiB
     - 93.4 MiB
     - 94.1 MiB
   * - 251
     - extremitypathfinder without Numba
     - 1.66 MiB
     - < 1 MiB
     - 30.4 MiB
     - 32.1 MiB
   * - 251
     - pyvisgraph
     - < 1 MiB
     - < 1 MiB
     - 20.9 MiB
     - 21.5 MiB
   * - 508
     - extremitypathfinder
     - 1.19 MiB
     - 1.69 MiB
     - 93.2 MiB
     - 94.4 MiB
   * - 508
     - extremitypathfinder without Numba
     - 2.72 MiB
     - 1.7 MiB
     - 30 MiB
     - 32.7 MiB
   * - 508
     - pyvisgraph
     - 1.91 MiB
     - 1.69 MiB
     - 20.8 MiB
     - 22.7 MiB
   * - 1,009
     - extremitypathfinder
     - 3.61 MiB
     - 3.53 MiB
     - 93.5 MiB
     - 97.2 MiB
   * - 1,009
     - extremitypathfinder without Numba
     - 7.94 MiB
     - 3.53 MiB
     - 30.1 MiB
     - 38 MiB
   * - 1,009
     - pyvisgraph
     - 3.77 MiB
     - 3.59 MiB
     - 20.8 MiB
     - 24.5 MiB
   * - 2,009
     - extremitypathfinder
     - 9.06 MiB
     - 6.87 MiB
     - 93.8 MiB
     - 103 MiB
   * - 2,009
     - extremitypathfinder without Numba
     - 15.3 MiB
     - 6.86 MiB
     - 30 MiB
     - 45.3 MiB
   * - 2,009
     - pyvisgraph
     - 11.7 MiB
     - 10.8 MiB
     - 21.1 MiB
     - 32.8 MiB
   * - 3,046
     - extremitypathfinder
     - 11.2 MiB
     - 9.15 MiB
     - 93.8 MiB
     - 105 MiB
   * - 3,046
     - extremitypathfinder without Numba
     - 16.2 MiB
     - 9.16 MiB
     - 30.6 MiB
     - 46.8 MiB
   * - 3,046
     - pyvisgraph
     - 23.3 MiB
     - 20.6 MiB
     - 21.3 MiB
     - 44.6 MiB
   * - 4,018
     - extremitypathfinder
     - 14.4 MiB
     - 10.8 MiB
     - 93.7 MiB
     - 108 MiB
   * - 4,018
     - extremitypathfinder without Numba
     - 21.7 MiB
     - 10.8 MiB
     - 30.6 MiB
     - 52.4 MiB
   * - 4,018
     - pyvisgraph
     - 42.2 MiB
     - 35 MiB
     - 21.3 MiB
     - 63.5 MiB
   * - 5,020
     - extremitypathfinder
     - 20.8 MiB
     - 16.1 MiB
     - 93.9 MiB
     - 115 MiB
   * - 5,020
     - extremitypathfinder without Numba
     - 26.6 MiB
     - 16.1 MiB
     - 30.5 MiB
     - 57.2 MiB
   * - 5,020
     - pyvisgraph
     - 31.1 MiB
     - 31.3 MiB
     - 21.8 MiB
     - 52.9 MiB
   * - 6,097
     - extremitypathfinder
     - 25.2 MiB
     - 20.1 MiB
     - 94.1 MiB
     - 119 MiB
   * - 6,097
     - extremitypathfinder without Numba
     - 30.6 MiB
     - 20.1 MiB
     - 31.1 MiB
     - 61.7 MiB
   * - 6,097
     - pyvisgraph
     - 47.3 MiB
     - 46.3 MiB
     - 21.8 MiB
     - 69.1 MiB

Baseline without input data (after the imports and the warm-up): extremitypathfinder with Numba 93.1 MiB, extremitypathfinder without Numba 29.4 MiB, pyvisgraph 20.5 MiB.
