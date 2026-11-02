.. list-table:: Peak memory of the build and the queries: above the baseline, the baseline (imports and warm-up) and the total
   :header-rows: 1

   * - Vertices
     - Above baseline extremitypathfinder
     - Baseline extremitypathfinder
     - Total extremitypathfinder
     - Above baseline pyvisgraph
     - Baseline pyvisgraph
     - Total pyvisgraph
   * - 249
     - 1.17 MiB
     - 86.3 MiB
     - 87.5 MiB
     - < 1 MiB
     - 18.1 MiB
     - 18.9 MiB
   * - 494
     - 2.5 MiB
     - 87.1 MiB
     - 89.6 MiB
     - 2.14 MiB
     - 17.9 MiB
     - 20 MiB
   * - 994
     - 7.12 MiB
     - 87.8 MiB
     - 94.9 MiB
     - 3.63 MiB
     - 18.1 MiB
     - 21.8 MiB
   * - 1,985
     - 19.2 MiB
     - 87.8 MiB
     - 107 MiB
     - 11.8 MiB
     - 18.3 MiB
     - 30.1 MiB
   * - 4,954
     - 34.2 MiB
     - 88.4 MiB
     - 123 MiB
     - 58.3 MiB
     - 18.9 MiB
     - 77.2 MiB
   * - 9,952
     - 61.9 MiB
     - 88.1 MiB
     - 150 MiB
     - –
     - –
     - –
   * - 19,956
     - –
     - –
     - –
     - –
     - –
     - –

Baseline without input data (after the imports and the warm-up): extremitypathfinder with Numba 86.5 MiB, extremitypathfinder without Numba 36.1 MiB, pyvisgraph 18.1 MiB.
