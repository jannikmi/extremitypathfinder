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
     - 110 ms
     - 505 ms
     - 4.59
     - 169 / 656
     - 249 / 1,945
     - 1.88 MiB
     - < 1 MiB
   * - 494
     - 428 ms
     - 2.14 s
     - 5
     - 336 / 1,539
     - 494 / 5,073
     - 4.73 MiB
     - 2.02 MiB
   * - 994
     - 1.64 s
     - 9.17 s
     - 5.58
     - 666 / 3,612
     - 994 / 14,957
     - 17.5 MiB
     - 3.92 MiB
   * - 1,985
     - 5.88 s
     - 36.8 s
     - 6.26
     - 1,245 / 7,212
     - 1,985 / 42,715
     - 58.3 MiB
     - 11.8 MiB
   * - 4,954
     - 32 s
     - invalid
     - –
     - 2,913 / 12,671
     - –
     - 309 MiB
     - –
   * - 9,952
     - 208 s
     - over budget
     - –
     - 6,593 / 108,626
     - –
     - 1.09 GiB
     - –
   * - 19,956
     - over budget
     - not run
     - –
     - –
     - –
     - –
     - –
