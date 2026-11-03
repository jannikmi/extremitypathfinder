.. list-table:: Preprocessing (build) time and graph size
   :header-rows: 1

   * - Vertices
     - Build extremitypathfinder
     - Build extremitypathfinder without Numba
     - Build pyvisgraph
     - Ratio pyvisgraph / extremitypathfinder
     - Graph extremitypathfinder (nodes / edges)
     - Graph pyvisgraph (nodes / edges)
   * - 249
     - 111 ms
     - 275 ms
     - 507 ms
     - 4.57
     - 169 / 656
     - 249 / 1,945
   * - 494
     - 423 ms
     - 1.1 s
     - 2.12 s
     - 5.02
     - 336 / 1,539
     - 494 / 5,073
   * - 994
     - 1.64 s
     - 4.14 s
     - 9 s
     - 5.48
     - 666 / 3,612
     - 994 / 14,957
   * - 1,985
     - 5.91 s
     - 15.3 s
     - 36.9 s
     - 6.24
     - 1,245 / 7,212
     - 1,985 / 42,715
   * - 4,954
     - 32.2 s
     - 87.6 s
     - 232 s
     - 7.23
     - 2,913 / 12,671
     - 4,954 / 277,886 (invalid)
   * - 9,952
     - 209 s
     - 462 s
     - over budget
     - –
     - 6,593 / 108,626
     - –
   * - 19,956
     - over budget
     - over budget
     - not run
     - –
     - –
     - –
