"""Offline decoder for finalized driver stage traces. No ROS or serial access."""
import argparse
import bisect
from collections import Counter, defaultdict
import csv
import json
from pathlib import Path
import statistics
import struct

HEADER=struct.Struct('<8sIIQQ')
RECORD=struct.Struct('<QQQQHHHHiId48s')
KINDS={1:'HW_READ_ENTER',2:'HW_READ_RESULT',3:'HW_STATE',4:'HW_WRITE_ENTER',5:'HW_GOAL',6:'HW_WRITE_EXIT',
       7:'DRIVER_READ_ENTER',8:'DRIVER_READ_EXIT',9:'DRIVER_WRITE_ENTER',10:'DRIVER_WRITE_EXIT',
       11:'GOAL_VALUE',12:'PACKED_GOAL',13:'RAW_POSITION',14:'CONVERTED_POSITION',
       15:'SERIAL_TX',16:'SERIAL_TX_RETURN',17:'SERIAL_RX'}


def records(path):
    with Path(path).open('rb') as f:
        h=f.read(HEADER.size)
        if len(h)!=HEADER.size:raise ValueError('Incomplete trace header')
        magic,size,pid,mono,wall=HEADER.unpack(h)
        if magic!=b'HX5TRC01' or size!=RECORD.size:raise ValueError('Unsupported trace format')
        while True:
            block=f.read(RECORD.size*4096)
            if not block:break
            if len(block)%RECORD.size:raise ValueError('Truncated record; preserve partial file')
            for item in RECORD.iter_unpack(block):yield (pid,mono,wall),item


def stats(values):
    if not values:return {'count':0}
    a=sorted(values)
    return {'count':len(a),'median':statistics.median(a),'p95':a[int((len(a)-1)*.95)],'max':a[-1]}


def crc16(data):
    crc=0
    for value in data:
        crc^=value<<8
        for _ in range(8):crc=((crc<<1)^0x8005)&65535 if crc&0x8000 else (crc<<1)&65535
    return crc


class Frames:
    """Reassemble only complete CRC-valid protocol2 frames; count discarded bytes."""
    def __init__(self):self.buffer=bytearray();self.bad_crc=0;self.discarded=0;self.total_bytes=0;self.frame_end_offsets=[]
    def feed(self,data):
        self.buffer.extend(data);self.total_bytes+=len(data);self.frame_end_offsets=[];result=[]
        while len(self.buffer)>=7:
            index=self.buffer.find(b'\xff\xff\xfd\x00')
            if index<0:
                self.discarded+=max(0,len(self.buffer)-3);self.buffer=self.buffer[-3:];break
            if index:self.discarded+=index;del self.buffer[:index]
            length=int.from_bytes(self.buffer[5:7],'little')+7
            if length<10 or length>65542:self.discarded+=1;del self.buffer[0];continue
            if len(self.buffer)<length:break
            frame=bytes(self.buffer[:length])
            if crc16(frame[:-2])!=int.from_bytes(frame[-2:],'little'):
                self.bad_crc+=1;del self.buffer[0];continue
            self.frame_end_offsets.append(self.total_bytes-len(self.buffer)+length)
            del self.buffer[:length];result.append(frame)
        return result


def analyse(path):
    path=Path(path);out=path.parent
    counts=Counter();durations=defaultdict(list);series=defaultdict(list);io_errors=[]
    rx=defaultdict(Frames);rx_ends=defaultdict(list);rx_meta=defaultdict(list);wire_counts=Counter();wire_rows=[];source_names=defaultdict(list)
    source_file=out/'sources.tsv'
    if source_file.exists():
        for line in source_file.read_text().splitlines():
            ident,name=line.split('\t',1);source_names[int(ident)].append(name)
    # All samples remain in the binary; CSV contains only changed scalar values.
    csv_path=out/(path.stem+'_changes.csv');last={}
    with csv_path.open('w',newline='') as file:
        writer=csv.writer(file);writer.writerow(['mono_ns','wall_anchor_ns','source','cycle','stage','board','motor','aux','result','value','payload_hex'])
        header=None
        for header,r in records(path):
            pid,mono,wall=header
            t,seq,source,cycle,kind,comm,ident,aux,result,size,value,data=r
            if size>48:raise ValueError('Invalid trace payload size')
            name=KINDS.get(kind,'UNKNOWN_'+str(kind));counts[name]+=1
            if kind in (8,10,16):durations[(source,name)].append(value)
            if kind in (2,6) and result:io_errors.append({'mono_ns':t,'source':source,'stage':name,'result':result})
            if kind==16 and result!=aux:io_errors.append({'mono_ns':t,'source':source,'stage':name,'requested':aux,'returned':result})
            if kind in (3,5,11,13,14):
                key=(source,comm,ident,kind);series[key].append((t,value))
                if last.get(key)!=value:
                    writer.writerow([t,wall+t-mono,source,cycle,name,comm,ident,aux,result,value,''])
                    last[key]=value
            elif kind==12:
                writer.writerow([t,wall+t-mono,source,cycle,name,comm,ident,aux,result,'',data[:size].hex()])
            elif kind==17:
                parser=rx[source]
                rx_ends[source].append(parser.total_bytes+size);rx_meta[source].append((t,cycle,wall+t-mono))
                frames=parser.feed(data[:size])
                for frame,end_offset in zip(frames,parser.frame_end_offsets):
                    # A lost observation can delay resynchronization. Timestamp each
                    # frame at the chunk that contained its last byte, not parse time.
                    actual_t,actual_cycle,actual_wall=rx_meta[source][bisect.bisect_left(rx_ends[source],end_offset)]
                    target=frame[4];body=frame[7:-2].replace(b'\xff\xff\xfd\xfd',b'\xff\xff\xfd')
                    wire_counts[(source,target)]+=1
                    if target in (110,140) and body and body[0]==0x55:
                        wire_rows.append({'mono_ns':actual_t,'wall_anchor_ns':actual_wall,'source':source,'cycle':actual_cycle,
                                          'board':target,'status_error':body[1],'payload_hex':body[2:].hex()})
        if header is None:raise ValueError('Trace contains no records')
    stage_delays={};raw_plateaus=[]
    for (source,comm,ident,kind),rows in series.items():
        rows.sort()
        if kind in (11,14,3):
            predecessor={11:5,14:13,3:14}[kind]
            before=series.get((source,comm,ident,predecessor),[])
            if before:
                before.sort();times=[t for t,_ in before];delays=[]
                for t,v in rows:
                    index=bisect.bisect_right(times,t)-1
                    if index>=0:
                        old_t,old_v=before[index]
                        # Raw encoded counts and radians differ, so that edge is time-only.
                        if kind==14 or abs(old_v-v)<=1e-8:delays.append((t-old_t)/1000)
                stage_delays[f'{source}/{comm}/{ident}/{KINDS[predecessor]}->{KINDS[kind]}_us']=stats(delays)
        if kind==13:
            goals=series.get((source,comm,ident,11),[])
            if not goals:continue
            goals.sort();gt=[t for t,_ in goals]
            begin=0
            for stop in range(1,len(rows)+1):
                if stop<len(rows) and rows[stop][1]==rows[begin][1]:continue
                segment=rows[begin:stop];a,b=segment[0][0],segment[-1][0]
                if b-a>=80000000:
                    lo=bisect.bisect_left(gt,a);hi=bisect.bisect_right(gt,b)
                    values=[v for _,v in goals[lo:hi]]
                    if values and max(values)-min(values)>=.1:
                        gaps=[(y[0]-x[0])/1e6 for x,y in zip(segment,segment[1:])]
                        raw_plateaus.append({'source':source,'board':comm,'motor':ident,'start_mono_ns':a,
                                             'duration_ms':(b-a)/1e6,'max_decoded_sample_gap_ms':max(gaps,default=0),
                                             'goal_excursion_rad':max(values)-min(values)})
                begin=stop
    status_path=path.with_suffix('.json')
    status=json.loads(status_path.read_text()) if status_path.exists() else {'missing_final_status':True}
    summary={'format':1,'header':{'pid':header[0],'mono_ns':header[1],'wall_ns':header[2]},
             'final_status':status,'counts':dict(counts),'sources':dict(source_names),
             'durations_us':{f'{source}/{kind}':stats(values) for (source,kind),values in durations.items()},
             'stage_delays':stage_delays,'read_write_result_errors':io_errors,
             'decoded_raw_plateaus':raw_plateaus,'wire_reply_counts':{f'{s}/{i}':n for (s,i),n in wire_counts.items()},
             'wire_crc_errors':{str(s):p.bad_crc for s,p in rx.items()},
             'limitations':['Host SDK write return is not motor application acknowledgement',
                            'RAW_POSITION is SDK-decoded getData, not a direct internal motor read',
                            'Use CRC-valid wire reply payloads to distinguish SDK cache from board reply',
                            'Trace drops invalidate incomplete chains; inspect final_status before attribution',
                            'Firmware/mechanical timing still requires physical observation or board-side trace']}
    (out/(path.stem+'_wire_replies.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in wire_rows))
    (out/(path.stem+'_analysis.json')).write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({'counts':summary['counts'],'final_status':status,'sources':summary['sources'],
                      'decoded_raw_plateaus':len(raw_plateaus),'wire_reply_counts':summary['wire_reply_counts']},indent=2))
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('path',type=Path)
    args=parser.parse_args();paths=[args.path] if args.path.is_file() else sorted(args.path.glob('**/trace_*.bin'))
    if not paths:raise SystemExit('No trace binary found')
    for p in paths:analyse(p)

if __name__=='__main__':main()
