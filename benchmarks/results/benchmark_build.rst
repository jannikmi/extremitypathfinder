.. list-table:: Preprocessing (build) time and graph size
   :header-rows: 1

   * - Vertices
     - Build extremitypathfinder
     - Build pyvisgraph
     - Ratio
     - Graph extremitypathfinder (nodes / edges)
     - Graph pyvisgraph (nodes / edges)
   * - 249
     - 115 ms
     - 510 ms
     - 4.41
     - 169 / 656
     - 249 / 1,945
   * - 494
     - 426 ms
     - 2.14 s
     - 5.03
     - 336 / 1,539
     - 494 / 5,073
   * - 994
     - 1.66 s
     - 8.98 s
     - 5.41
     - 666 / 3,612
     - 994 / 14,957
   * - 1,985
     - 5.9 s
     - 36.9 s
     - 6.26
     - 1,245 / 7,212
     - 1,985 / 42,715
   * - 4,954
     - 32.2 s
     - 234 s
     - 7.26
     - 2,913 / 12,671
     - 4,954 / 277,886 (invalid)
   * - 9,952
     - 215 s
     - over budget
     - –
     - 6,593 / 108,626
     - –
   * - 19,956
     - over budget
     - not run
     - –
     - –
     - –
