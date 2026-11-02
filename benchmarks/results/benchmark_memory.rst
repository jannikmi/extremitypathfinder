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
     - 1.09 MiB
     - 88 MiB
     - 89 MiB
     - < 1 MiB
     - 17.7 MiB
     - 18.4 MiB
   * - 494
     - 3.02 MiB
     - 87.5 MiB
     - 90.5 MiB
     - 1.62 MiB
     - 17.8 MiB
     - 19.4 MiB
   * - 994
     - 8.53 MiB
     - 86.7 MiB
     - 95.2 MiB
     - 4.14 MiB
     - 17.8 MiB
     - 21.9 MiB
   * - 1,985
     - 19.3 MiB
     - 86.7 MiB
     - 106 MiB
     - 11.5 MiB
     - 18.2 MiB
     - 29.7 MiB
   * - 4,954
     - 34.6 MiB
     - 87.1 MiB
     - 122 MiB
     - 57.8 MiB
     - 18.8 MiB
     - 76.6 MiB
   * - 9,952
     - 70.6 MiB
     - 89.8 MiB
     - 160 MiB
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

Baseline without input data (after the imports and the warm-up): extremitypathfinder with Numba 87.5 MiB, extremitypathfinder without Numba 35.8 MiB, pyvisgraph 18 MiB.
