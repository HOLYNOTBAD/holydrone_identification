#!/usr/bin/env python3
"""Inspect independent stepped-sine flights using actual rotor speed and PX4 gyro.
The frequency response identifies km/Izz and Dz/Izz, not their absolute scale.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
import yaml


def sinusoid(t, signal, frequency):
    phase=2*np.pi*frequency*t
    design=np.column_stack([np.cos(phase),np.sin(phase),np.ones(len(t)),t-t.mean()])
    values=np.linalg.lstsq(design,signal,rcond=None)[0]
    residual=signal-design@values
    return complex(values[0],-values[1]), float(np.sqrt(np.mean(residual**2)))


def response_model(f, a, b, delay):
    jw=2j*np.pi*np.asarray(f)
    return a/(b+jw)*np.exp(-jw*delay)


def fit_ratio(rows):
    f=np.array([r['frequency_hz'] for r in rows])
    observed=np.array([complex(r['real'],r['imag']) for r in rows])
    def residual(v):
        delta=(response_model(f,*v)-observed)/np.abs(observed)
        return np.r_[delta.real,delta.imag]
    fit=least_squares(residual,[8.,.1,.01],bounds=([.01,0,0],[1000,100,.15]))
    return dict(km_over_izz=float(fit.x[0]), damping_over_izz=float(fit.x[1]),
                effective_delay_s=float(fit.x[2]), normalized_complex_rmse=float(np.sqrt(np.mean(fit.fun**2))),
                at_boundary=bool(np.any(fit.active_mask!=0)),converged=bool(fit.success))


def main():
    from pyulog import ULog
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True)
    args=p.parse_args(); root=args.root
    report=yaml.safe_load((root/'physics_report.yaml').read_text())
    kf=report['propulsion']['gazebo_motor_constant_n_per_rad_s_squared']
    signs=np.array(report['measured']['yaw_torque_signs'])
    output={}
    for name in ['physics_train','physics_validation']:
        path=root/name
        events=[json.loads(line) for line in (path/'events.jsonl').read_text().splitlines()]
        starts={}
        for e in events:
            if e['phase']=='pulse' and e.get('sine_hz',0)>0:
                starts.setdefault(e['segment'],e)
        csv_data=np.genfromtxt(path/'motor_speed.csv',delimiter=',',names=True)
        omega=np.column_stack([csv_data[f'omega_{i}_rad_s'] for i in range(4)])
        delta_force=kf*(omega**2@signs)
        log=ULog(str(next((path/'px4/log').rglob('*.ulg'))),['vehicle_angular_velocity'])
        gyro=next(d.data for d in log.data_list if d.name=='vehicle_angular_velocity' and d.multi_id==0)
        t=gyro['timestamp_sample']*1e-6; r=-gyro['xyz[2]']
        rows=[]
        for segment,event in starts.items():
            f=event['sine_hz']; begin=event['t']; end=begin+event['pulse_hold_s']
            # Discard the first cycle; leave a 20 ms end margin for the status/pose timestamp difference.
            selected=(t>=begin+1/f)&(t<end-.02)
            ts=t[selected]; y=r[selected]
            if len(ts)<30 or np.max(np.diff(ts))>.03:
                raise ValueError('Insufficient contiguous gyro samples in sine segment')
            if ts[0]<csv_data['t_ros_s'][0] or ts[-1]>csv_data['t_ros_s'][-1]:
                raise ValueError('RPM does not cover sine segment')
            u=np.interp(ts,csv_data['t_ros_s'],delta_force)
            force,_=sinusoid(ts-ts[0],u,f); rate,error=sinusoid(ts-ts[0],y,f)
            if abs(force)<1e-4: raise ValueError('Insufficient yaw torque excitation')
            response=rate/force
            rows.append(dict(segment=segment,frequency_hz=f,input_amplitude_n=abs(force),
                rate_amplitude_rad_s=abs(rate),rate_non_sinusoidal_rmse_rad_s=error,
                real=response.real,imag=response.imag,gain=abs(response),phase_deg=float(np.angle(response,deg=True))))
        if len(rows)<4: raise ValueError('Need at least four swept segments')
        output[name]=dict(segments=rows,ratio_model=fit_ratio(rows),ulog_dropouts=len(log.dropouts))
    training=output['physics_train']['ratio_model']
    validation=output['physics_validation']['segments']
    observed=np.array([complex(r['real'],r['imag']) for r in validation])
    predicted=response_model([r['frequency_hz'] for r in validation],training['km_over_izz'],training['damping_over_izz'],training['effective_delay_s'])
    output['held_out_normalized_complex_error']=float(np.sqrt(np.mean(np.abs((predicted-observed)/observed)**2)))
    output['scope']='Effective yaw frequency response; cannot separate Izz and km. ROS-PX4 offset is zero only for these lockstep SITL flights.'
    (root/'sweep_response.json').write_text(json.dumps(output,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(8,6),sharex=True)
    for name in ['physics_train','physics_validation']:
        rows=output[name]['segments']
        axes[0].scatter([r['frequency_hz'] for r in rows],[r['gain'] for r in rows],label=name)
        axes[1].scatter([r['frequency_hz'] for r in rows],[r['phase_deg'] for r in rows],label=name)
    fs=np.geomspace(.5,3,100); prediction=response_model(fs,training['km_over_izz'],training['damping_over_izz'],training['effective_delay_s'])
    axes[0].plot(fs,np.abs(prediction),label='training model');axes[1].plot(fs,np.angle(prediction,deg=True))
    axes[0].set_ylabel('Yaw rate / differential force [(rad/s)/N]');axes[0].set_yscale('log');axes[0].legend()
    axes[1].set_ylabel('Phase [deg]');axes[1].set_xlabel('Frequency [Hz]');axes[1].set_xscale('log')
    for ax in axes: ax.grid(True,alpha=.3)
    fig.tight_layout();fig.savefig(root/'sweep_response.png',dpi=160);plt.close(fig)
    print(json.dumps({name:output[name]['ratio_model'] for name in ['physics_train','physics_validation']},indent=2))
    print('Held-out normalized complex error:',output['held_out_normalized_complex_error'])


if __name__=='__main__': main()
