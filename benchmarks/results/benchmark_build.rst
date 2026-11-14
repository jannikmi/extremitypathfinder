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
     - 66.4 ms
     - 103 ms
     - 512 ms
     - 7.72
     - 170 / 663
     - 251 / 1,974
   * - 508
     - 240 ms
     - 383 ms
     - 2.25 s
     - 9.35
     - 345 / 1,569
     - 508 / 5,216
   * - 1,009
     - 856 ms
     - 1.38 s
     - 9.47 s
     - 11.1
     - 676 / 3,678
     - 1,009 / 15,352
   * - 2,009
     - 2.86 s
     - 4.99 s
     - 38.8 s
     - 13.6
     - 1,262 / 7,267
     - 2,009 / 43,992
   * - 3,046
     - 6.2 s
     - 10.2 s
     - 87.4 s
     - 14.1
     - 1,848 / 10,005
     - 3,046 / 92,119 (invalid)
   * - 4,018
     - 9.82 s
     - 17.1 s
     - 153 s
     - 15.6
     - 2,393 / 11,768
     - 4,018 / 163,646 (invalid)
   * - 5,020
     - 18.6 s
     - 31.6 s
     - 259 s
     - 13.9
     - 3,214 / 37,384
     - 5,020 / 156,798 (invalid)
   * - 6,097
     - 27.7 s
     - 47 s
     - 386 s
     - 14
     - 3,940 / 52,765
     - 6,097 / 223,226 (invalid)
