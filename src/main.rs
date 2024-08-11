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

#[derive(Debug, PartialEq, Clone, Default)]
#[repr(C)]
struct RadarPoint{
    x: f32,
    y: f32,
    z: f32,

    velocity: f32,
    range: f32,
    theta: f32,
    phi: f32,

    rcs: f32,
    snr: f32,
    amplitude: f32,
    power: f32,

    az_conf: i32,
    el_conf: i32,
    
    af_type: i32,
    af_az_type: i32,
    af_el_type: i32,

    rdd_idx: i32,

    std_range: f32,
    std_vel: f32,
    std_theta: f32,
    std_phi: f32,
}

impl RadarPoint {
    fn new(
        x: f32,
        y: f32,
        z: f32,

        velocity: f32,
        range: f32,
        theta: f32,
        phi: f32,

        rcs: f32,
        snr: f32,
        amplitude: f32,
        power: f32,

        az_conf: i32,
        el_conf: i32,
        
        af_type: i32,
        af_az_type: i32,
        af_el_type: i32,

        rdd_idx: i32,

        std_range: f32,
        std_vel: f32,
        std_theta: f32,
        std_phi: f32,

    ) -> Self {
        Self {
            x,
            y,
            z,

            velocity,
            range,
            theta,
            phi,

            rcs,
            snr,
            amplitude,
            power,

            az_conf,
            el_conf,

            af_type,
            af_az_type,
            af_el_type,

            rdd_idx,

            std_range,
            std_vel,
            std_theta,
            std_phi,
        }
    }
}

impl From<RadarPoint> for RPCL2Point<21> {
    fn from(point: RadarPoint) -> Self {
        [
            point.x.into(),
            point.y.into(),
            point.z.into(),

            point.velocity.into(),
            point.range.into(),
            point.theta.into(),
            point.phi.into(),

            point.rcs.into(),
            point.snr.into(),
            point.amplitude.into(),
            point.power.into(),

            point.az_conf.into(),
            point.el_conf.into(),

            point.af_type.into(),
            point.af_az_type.into(),
            point.af_el_type.into(),

            point.rdd_idx.into(),

            point.std_range.into(),
            point.std_vel.into(),
            point.std_theta.into(),
            point.std_phi.into(),
        ]
        .into()
    }
}

impl From<RPCL2Point<21>> for RadarPoint {
    fn from(point: RPCL2Point<21>) -> Self {
        Self::new(
            point[0].get(),   // x
            point[1].get(),   // y
            point[2].get(),   // z
            point[3].get(),   // velocity
            point[4].get(),   // range
            point[5].get(),   // theta
            point[6].get(),   // phi
            point[7].get(),   // rcs
            point[8].get(),   // snr
            point[9].get(),   // amplitude
            point[10].get(),  // power
            point[11].get(),  // az_conf
            point[12].get(),  // el_conf
            point[13].get(),  // af_type
            point[14].get(),  // af_az_type
            point[15].get(),  // af_el_type
            point[16].get(),  // rdd_idx
            point[17].get(),  // std_range
            point[18].get(),  // std_vel
            point[19].get(),  // std_theta
            point[20].get(),  // std_phi
        )
    }
}


impl PointConvertible<21> for RadarPoint {
    fn layout() -> LayoutDescription {
        LayoutDescription::new(&[
            LayoutField::new("x", "f32", 4),
            LayoutField::new("y", "f32", 4),
            LayoutField::new("z", "f32", 4),

            LayoutField::new("velocity", "f32", 4),
            LayoutField::new("range", "f32", 4),
            LayoutField::new("theta", "f32", 4),
            LayoutField::new("phi", "f32", 4),

            LayoutField::new("rcs", "f32", 4),
            LayoutField::new("snr", "f32", 4),
            LayoutField::new("amplitude", "f32", 4),
            LayoutField::new("power", "f32", 4),

            LayoutField::new("az_conf", "i32", 4),
            LayoutField::new("el_conf", "i32", 4),
            
            LayoutField::new("af_type", "i32", 4),
            LayoutField::new("af_az_type", "i32", 4),
            LayoutField::new("af_el_type", "i32", 4),

            LayoutField::new("rdd_idx", "i32", 4),

            LayoutField::new("std_range", "f32", 4),
            LayoutField::new("std_vel", "f32", 4),
            LayoutField::new("std_theta", "f32", 4),
            LayoutField::new("std_phi", "f32", 4),
        ])
    }
}




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

        //println!("length: {0}", slice.len());

        data_buffer.extend_from_slice(&slice);

        if header.stream_chunk_idx == 73 {

            //println!("lenghth: {0}", data_buffer.len());

            let stream_data: StreamData = deserialize(&data_buffer).unwrap();

            //println!("array {:?}", stream_data.af_data_ran);

            
            let mut cloud_points = Vec::new();
            for i in 0..stream_data.af_data_num_af_det  as usize{
                
                if !(stream_data.af_data_snr[i] > 0.0){
                    //println!("Point is zero");
                    break;
                }
                //println!("Range: {:.16}", stream_data.af_data_ran[i]);
                //println!("Theta: {:.16}", stream_data.af_data_theta[i]);
                //println!("Velocity: {:.8}", stream_data.af_data_vel[i]);
                //println!("Result: {:.16}", stream_data.af_data_theta[i].sin()*stream_data.af_data_ran[i]);

                    
                cloud_points.push(RadarPoint {
                    x: stream_data.af_data_ran[i],
                    y: stream_data.af_data_theta[i].sin()*stream_data.af_data_ran[i],
                    z: stream_data.af_data_phi[i].sin()*stream_data.af_data_ran[i],

                    velocity: stream_data.af_data_vel[i],
                    range: stream_data.af_data_ran[i],
                    theta: stream_data.af_data_theta[i],
                    phi: stream_data.af_data_phi[i],

                    rcs: stream_data.af_data_rcs[i],
                    snr: stream_data.af_data_snr[i],
                    amplitude: stream_data.af_data_amp[i],
                    power: stream_data.af_data_pow[i],

                    az_conf: stream_data.af_data_az_conf[i],
                    el_conf: stream_data.af_data_el_conf[i],

                    af_type: stream_data.af_data_af_type[i],
                    af_az_type: stream_data.af_data_az_af_type[i],
                    af_el_type: stream_data.af_data_el_af_type[i],

                    rdd_idx: stream_data.af_data_rdd_idx[i],

                    std_range: stream_data.af_data_std_vel[i],
                    std_vel: stream_data.af_data_std_vel[i],
                    std_theta: stream_data.af_data_std_theta[i],
                    std_phi: stream_data.af_data_std_phi[i],

                });
            }
            
            let msg = PointCloud2Msg::try_from_iter(cloud_points).unwrap();

            let r2r_msg: PointCloud2 = msg.into();

            publisher.publish(&r2r_msg).unwrap();

            data_buffer = Vec::new();
        }
        
    }
}