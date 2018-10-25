# Shenango

Shenango is a system that enables servers in datacenters to
simultaneously provide low tail latency and high CPU efficiency, by
rapidly reallocating cores across applications, at timescales as small
as every 5 microseconds.

## How do I use it?

1) Clone the Shenango repository.

```
git clone https://github.com/abelay/shenango
cd shenango
```

2) Setup DPDK and build the IOKernel and Shenango runtime.

```
./dpdk.sh
./scripts/setup_machine.sh
make clean && make
```

To use Mellanox NICs, build with `make MLX=1`. You can also build with
debugging enabled (`make DEBUG=1`).

3) Install Rust and build a synthetic client-server application.

```
curl https://sh.rustup.rs -sSf | sh
rustup default nightly
```
```
cd apps/synthetic
cargo clean
cargo update
cargo build --release
```

4) Run the synthetic application with a client and server. The client
sends requests to the server, which performs a specified amount of
fake work (e.g., computing square roots for 10us), before responding.

On the server:
```
sudo ./iokerneld
./apps/synthetic/target/release/synthetic 192.168.1.3:5000 --config server.config --mode spawner-server
```

On the client:
```
sudo ./iokerneld
./apps/synthetic/target/release/synthetic 192.168.1.3:5000 --config client.config --mode runtime-client
```

## How do I contribute?

### Code Overview

apps - synthetic and benchmarking applications.

base - a extension to the standard C library that provides tools for managing
lists, memory, bitmaps, initialization, atomics, and several other useful
features.

bindings - language bindings (C++ and rust) for the runtime.

dpdk - [DPDK](https://www.dpdk.org/) library for accessing NIC queues
from userspace.

dune - a better implementation of libdune based on the base library.

iokernel - dedicated core that steers packets and reallocates cores
across applications.

net - a packet manipulation library.

runtime - a user-level threading and networking runtime.

shim - a shim layer that enables running unmodified
[PARSEC](http://parsec.cs.princeton.edu/) applications atop Shenango.


### Coding Style

Use the following conventions for C code:
https://www.kernel.org/doc/html/v4.10/process/coding-style.html

Use the following conventions for C++ code:
https://google.github.io/styleguide/cppguide.html

For third party libraries and tools, use their existing coding style.

For some helpful tips on how to write clean code, see:
https://www.lysator.liu.se/c/pikestyle.html
