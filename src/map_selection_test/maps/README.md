# Map files

`test.yaml` is committed here. **`test.pgm` is not** - it is a binary map image
and must be copied in from the machine that recorded it:

```bash
cp ~/ros2_ws/src/map_selection_test/maps/test.pgm \
   ~/beetlebot_ws/src/map_selection_test/maps/test.pgm
```

`test.yaml` refers to the image as `image: test.pgm`, so the two files must sit
in the same directory. Both are installed into
`share/map_selection_test/maps/` by `setup.py`, which is why nothing needs an
absolute path into a home directory.

The build does not fail if `test.pgm` is missing (the install list is a glob);
the node reports `Failed to load map image` at run time instead.
