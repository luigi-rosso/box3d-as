# Upstream Box3D issues

Found while porting against `f36d061`. The port reproduces each one exactly,
since it stays faithful; the fix belongs upstream.

## b3CreateHull crashes or hangs on near coplanar clouds

2D points uniform in [-1, 1]^2 with z uniform in [-1e-5, 1e-5]: the release
build segfaults in `b3NewellPlane` on some seeds and never returns on others.
A debug build asserts at hull.c:544 (`twin->twin == edge`, reached from
`b3HullBuilder_ConnectEdges` in `ConnectFaces` in `MergeCoplanar`) or at
hull.c:1093 (`b3CheckConsistency` after `ConnectFaces`). In the hangs
`ConnectEdges` merges a face with itself. Repro and seeds in `notes/hull.md`.
Anyone building hulls from user or asset data can hit it.
