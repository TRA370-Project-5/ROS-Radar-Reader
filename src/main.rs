use std::net::UdpSocket;
use std::mem;
use bincode::deserialize;
use serde::Deserialize;

extern crate serde_big_array;
use serde_big_array::BigArray;

use ros_pointcloud2::prelude::*;

use r2r::{
    QosProfile, sensor_msgs::msg::PointCloud2,
};



#[repr(packed)]
struct MudpHeaderA5Tag {
/* Application Layer Info */
    version_info: u16,  /* Version of this header specification [set to RECORD_VERSIONINFO] */
    source_tx_cnt: u16,  /* Incremented on every transmission from this source.  */
    source_tx_time: u32, /* Timestamp (ms) when packet queued for transmit. */
    source_info: u8 ,    /* Definition not available yet */
    source_info1: u8,
    source_info2: u8,
    source_info3: u8,

   /* Process Layer Info */
    stream_ref_index: u32,          /* Stream-specific index used for data retrieval (e.g. Look Index for radar) */
    stream_data_len: u16,           /* Number of bytes in the current message�s payload, not including the record header */
    stream_tx_cnt: u8,              /* Increments on each transmission of associated stream number. */
    stream_number: u8,              /* Stream number [0:FF]. One source may transmit multiple, asynchronous streams. */
    stream_version: u8,             /* Stream data structure format/version (how to interpret data). */
    stream_chunks_per_cycle: u8,    /* Stream chunks per cycle, applicable for static streams e.g. calibration */
    stream_chunks: u16,             /* A Stream may be transmitted from the process layer as M equal-size
                                     chunks that can be reconstructed on the receive side.
                                     Set to 0 if transmission is not part of a series of chunks (normal mode).
                                     Set to M (>=2) to indicate stream is sent as a series of M chunks.
                                     NOTE: streamTxCnt must increment for each chunk while streamRefIndex
                                     would be the same for each chunk of a series. */
    stream_chunk_idx: u16,          /* Zero when streamChunks is zero. Otherwise, represents this chunk's
                                     position in the reconstruction queue [0:(streamChunks-1)]. */
    reserved: u8,                   /* Set to zero */
    sensor_id: u8,                  /* Sensor Position ID */
}


#[derive(Deserialize, Debug)]
pub struct StreamData {
    stream_hdr_size: u32,
    stream_hdr_version: u16,
    stream_hdr_checksum: u16,
    stream_hdr_scan_index: u16,
    stream_hdr_error_info: u16,
    stream_hdr_module_time_ms: u32,
    bistatic_count: u32,

    af_xput_count: f32,
    af_xput_inst: f32,
    af_data_num_af_det: i32,
    #[serde(with = "BigArray")]
    af_data_ran: [f32; 1024],

    #[serde(with = "BigArray")]
    af_data_vel: [f32; 1024],
    #[serde(with = "BigArray")]
    af_data_pow: [f32; 1024],
    #[serde(with = "BigArray")]
    af_data_snr: [f32; 1024],
    #[serde(with = "BigArray")]
    af_data_theta: [f32; 1024],
    #[serde(with = "BigArray")]
    af_data_phi: [f32; 1024],
    #[serde(with = "BigArray")]
    af_data_rcs: [f32; 1024],
    #[serde(with = "BigArray")]
    af_data_amp: [f32; 1024],
    #[serde(with = "BigArray")]
    af_data_sin_theta3: [f32; 1024],
    #[serde(with = "BigArray")]
    af_data_sin_phi3: [f32; 1024],
    #[serde(with = "BigArray")]
    af_data_az_conf: [i32; 1024],
    #[serde(with = "BigArray")]
    af_data_el_conf: [i32; 1024],
    #[serde(with = "BigArray")]
    af_data_f_single_target: [i32; 1024],
    #[serde(with = "BigArray")]
    af_data_f_superres_target: [i32; 1024],
    #[serde(with = "BigArray")]
    af_data_f_laz2el: [i32; 1024],
    #[serde(with = "BigArray")]
    af_data_f_bistatic: [i32; 1024],
    #[serde(with = "BigArray")]
    af_data_f_peak_det: [i32; 1024],
    #[serde(with = "BigArray")]
    af_data_af_type: [i32; 1024],
    #[serde(with = "BigArray")]
    af_data_az_af_type: [i32; 1024],
    #[serde(with = "BigArray")]
    af_data_el_af_type: [i32; 1024],
    #[serde(with = "BigArray")]
    af_data_rdd_idx: [i32; 1024],
    #[serde(with = "BigArray")]
    af_data_spec_res: [f32; 1024],
    #[serde(with = "BigArray")]
    af_data_std_ran: [f32; 1024],
    #[serde(with = "BigArray")]
    af_data_std_vel: [f32; 1024],
    #[serde(with = "BigArray")]
    af_data_std_theta: [f32; 1024],
    #[serde(with = "BigArray")]
    af_data_std_phi: [f32; 1024],

    c66_error_code: u16,
    look_index: u16,
    scan_index: u16,
    trigger_missed_cnt: u16,
    look_type: u16,
    range_coverage: u16,
    doppler_coverage: i16,
    rest_count: u16,
    num_fp_detections: u16,
    num_sp_detections: u16,
    k_unused16_1: u16,
    xcp_mode: u8,
    k_unused8_1: u8,

}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "radar_reader", "")?;

    let publisher = node.create_publisher::<PointCloud2>("/pointcloud", QosProfile::default())?;

    println!("node name: {}", node.name()?);
    println!("node fully qualified name: {}", node.fully_qualified_name()?);

    let socket: UdpSocket = UdpSocket::bind("192.168.1.40:5555")?;
    
    let mut buf: [u8; 1472] = [0; 1472];
    let mut prev_chunk_idx: u16 = 0;

    let mut data_buffer = Vec::new();

    loop {
        let (data_length, src) = socket.recv_from(&mut buf)?;

        let header: MudpHeaderA5Tag = unsafe{
            let header: MudpHeaderA5Tag = mem::transmute_copy(&buf);
            
            header

        };
        
        if header.stream_number != 1 {
            continue;
        }

        if header.stream_chunk_idx != 0 && header.stream_chunk_idx != prev_chunk_idx + 1 {
            prev_chunk_idx = 0;

            data_buffer= Vec::new();
            continue;
        }

        prev_chunk_idx = header.stream_chunk_idx;
        let chunk_idx = header.stream_chunk_idx;

        let slice = &buf[28..1472];

        data_buffer.extend_from_slice(&slice);

        if header.stream_chunk_idx == 73 {
            for _ in 0..66 {
                data_buffer.push(0);
            }

            let stream_data: StreamData = deserialize(&data_buffer).unwrap();

            
            let mut cloud_points = Vec::new();
            for i in 0..stream_data.af_data_num_af_det  as usize{
                
                if !(stream_data.af_data_snr[i] > 0.0){
                    println!("Point is zero");
                    break;
                }
                println!("Range: {:.16}", stream_data.af_data_ran[i]);
                println!("Theta: {:.16}", stream_data.af_data_theta[i]);
                println!("Velocity: {:.8}", stream_data.af_data_vel[i]);
                println!("Result: {:.16}", stream_data.af_data_theta[i].sin()*stream_data.af_data_ran[i]);
                    
                cloud_points.push(PointXYZ {
                    x: stream_data.af_data_ran[i],
                    y: stream_data.af_data_theta[i].sin()*stream_data.af_data_ran[i],
                    z: stream_data.af_data_phi[i].sin()*stream_data.af_data_ran[i],
                });
            }
            
            let msg = PointCloud2Msg::try_from_iter(cloud_points).unwrap();

            let r2r_msg: PointCloud2 = msg.into();

            publisher.publish(&r2r_msg).unwrap();

            data_buffer = Vec::new();
        }
        
    }
}

