use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use clap::Arg;
use std::cmp::min;
use std::collections::HashMap;
use std::io;
use std::io::Read;

use crate::Buffer;
use crate::Connection;
use crate::LoadgenProtocol;
use crate::Packet;
use crate::Transport;

#[derive(Copy, Clone)]
pub struct InvokeProtocol {
    args: &'static str,
}

impl InvokeProtocol {
    pub fn with_args(matches: &clap::ArgMatches, tport: Transport) -> Self {
        if let Transport::Udp = tport {
            panic!("udp is unsupported");
        }

        let arg = matches.value_of("function_args").unwrap();


        InvokeProtocol {
            args: Box::leak(arg.to_string().into_boxed_str())
        }
    }

    pub fn args<'a, 'b>() -> Vec<clap::Arg<'a, 'b>> {
        vec![Arg::with_name("function_args")
            .long("function_args")
            .takes_value(true)
            .default_value("/tmp/args.json")
            .help("path to json formatted function arguments")]
    }
}

impl LoadgenProtocol for InvokeProtocol {
    fn uses_ordered_requests(&self) -> bool {
        true
    }

    fn gen_req(&self, _i: usize, _p: &Packet, buf: &mut Vec<u8>) {
        buf.write_u64::<LittleEndian>(self.args.len() as u64).unwrap();
        buf.extend(self.args.as_bytes());
    }

    fn read_response(&self, mut sock: &Connection, buf: &mut Buffer) -> io::Result<(usize, u64)> {
        let scratch = buf.get_empty_buf();
        let mut size = sock.read_u64::<LittleEndian>()? as usize;

        while size > 0 {
            let rlen = min(size, scratch.len());
            sock.read_exact(&mut scratch[..rlen])?;
            size -= rlen;
        }

        return Ok((0, 0));
    }
}
