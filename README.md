# Genesis for Analogue Pocket

This is a port of a mister port of the [fpgagen](https://github.com/Torlus/fpgagen) core for Analogue Pocket.
I am sure I am forgetting something else here... I know some JT and Kitrinx modules are used. So, shout out to them.

- Various modules by [Jotego](https://www.patreon.com/topapate) are used
- Composite mode module is by [Kitrinx](https://github.com/Kitrinx)
- Many improvements from [sorgelig](https://github.com/sorgelig)
- Many improvements from [srg320](https://github.com/srg320)
- Various modules by [agg23](https://github.com/agg23)
- Special thanks to [tpwrules](https://github.com/tpwrules)

fpgagen - a SEGA Megadrive/Genesis clone in a FPGA.
Copyright (c) 2010-2013 Gregory Estrade (greg@torlus.com)
All rights reserved

## Important to read this first!
- This core is far from complete and I am aware of most issues.
- No PAL.

## Building

If you don't want to install Intel Quartus on your system, you can use Docker to
build everything in isolation.

First, [install Docker](https://docs.docker.com/get-started/get-docker/) of course.

Next, from the root of this checkout, compile the core.
NB: Depending on the speed of your machine, this can take a while, easily 30min+.

```
$ cd src/fpga
$ docker run --rm -v .:/build raetro/quartus:21.1 quartus_sh --flow compile ap_core
```

Now reverse the bitstream, and copy it to the output.

```
$ ./output_files/reverse_bits.py output_files/ap_core.rbf ../../dist/Cores/ericlewis.Genesis/bitstream.rbf_r
```

Now you can zip up the dist/ tree, or manually install those files to your
Pocket's SD card.
