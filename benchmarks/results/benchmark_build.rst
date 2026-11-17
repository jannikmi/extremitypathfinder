.. list-table:: Preprocessing (build) time and graph size
   :header-rows: 1

   * - Vertices
     - Build extremitypathfinder
     - Build extremitypathfinder without Numba
     - Build pyvisgraph
     - Ratio pyvisgraph / extremitypathfinder
     - Graph extremitypathfinder (nodes / edges)
     - Graph pyvisgraph (nodes / edges)
   * - 251
     - 29.1 ms
     - 60 ms
     - 513 ms
     - 17.7
     - 170 / 663
     - 251 / 1,974
   * - 508
     - 75.6 ms
     - 198 ms
     - 2.24 s
     - 29.7
     - 345 / 1,569
     - 508 / 5,216
   * - 1,009
     - 204 ms
     - 637 ms
     - 9.22 s
     - 45.3
     - 676 / 3,678
     - 1,009 / 15,352
   * - 2,009
     - 555 ms
     - 2.24 s
     - 37.6 s
     - 67.9
     - 1,262 / 7,267
     - 2,009 / 43,992
   * - 3,046
     - 1.04 s
     - 4.67 s
     - 87.3 s
     - 83.7
     - 1,848 / 10,005
     - 3,046 / 92,119 (invalid)
   * - 4,018
     - 1.59 s
     - 7.72 s
     - 152 s
     - 95.5
     - 2,393 / 11,768
     - 4,018 / 163,646 (invalid)
   * - 5,020
     - 3.38 s
     - 14.3 s
     - 257 s
     - 76.2
     - 3,214 / 37,384
     - 5,020 / 156,798 (invalid)
   * - 6,097
     - 4.91 s
     - 21 s
     - 383 s
     - 78.1
     - 3,940 / 52,765
     - 6,097 / 223,226 (invalid)
