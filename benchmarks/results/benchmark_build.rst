.. list-table:: Preprocessing (build) time, graph size and peak memory
   :header-rows: 1

   * - Vertices
     - Build extremitypathfinder
     - Build pyvisgraph
     - Ratio
     - Graph extremitypathfinder (nodes / edges)
     - Graph pyvisgraph (nodes / edges)
     - Memory extremitypathfinder
     - Memory pyvisgraph
   * - 249
     - 115 ms
     - 527 ms
     - 4.56
     - 169 / 656
     - 249 / 1,945
     - 1.19 MiB
     - < 1 MiB
   * - 494
     - 452 ms
     - 2.22 s
     - 4.92
     - 336 / 1,539
     - 494 / 5,073
     - 2.81 MiB
     - 2.16 MiB
   * - 994
     - 1.76 s
     - 9.41 s
     - 5.33
     - 666 / 3,612
     - 994 / 14,957
     - 7.05 MiB
     - 3.98 MiB
   * - 1,985
     - 6.26 s
     - 38.9 s
     - 6.22
     - 1,245 / 7,212
     - 1,985 / 42,715
     - 19.4 MiB
     - 11.4 MiB
   * - 4,954
     - 34.3 s
     - invalid
     - –
     - 2,913 / 12,671
     - –
     - 33.5 MiB
     - –
   * - 9,952
     - 211 s
     - over budget
     - –
     - 6,593 / 108,626
     - –
     - 74 MiB
     - –
   * - 19,956
     - over budget
     - not run
     - –
     - –
     - –
     - –
     - –
