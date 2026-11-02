.. list-table:: Preprocessing (build) time and graph size
   :header-rows: 1

   * - Vertices
     - Build extremitypathfinder
     - Build pyvisgraph
     - Ratio
     - Graph extremitypathfinder (nodes / edges)
     - Graph pyvisgraph (nodes / edges)
   * - 249
     - 118 ms
     - 521 ms
     - 4.41
     - 169 / 656
     - 249 / 1,945
   * - 494
     - 436 ms
     - 2.19 s
     - 5.02
     - 336 / 1,539
     - 494 / 5,073
   * - 994
     - 1.71 s
     - 9.38 s
     - 5.48
     - 666 / 3,612
     - 994 / 14,957
   * - 1,985
     - 6.18 s
     - 38.4 s
     - 6.21
     - 1,245 / 7,212
     - 1,985 / 42,715
   * - 4,954
     - 34 s
     - 242 s
     - 7.13
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
