#!/usr/bin/env python3
"""Compare a completed SITL physics report with captured Gazebo link properties and SDF.
Usage: python compare_gazebo_truth.py --session-root DATA --sdf MODEL.sdf
Generates known-mass fit config even before physics_report.yaml exists; never writes SDF.
"""
import argparse
import csv
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-root', type=Path, required=True)
    parser.add_argument('--sdf', type=Path, required=True)
    args = parser.parse_args()
    root = args.session_root
    links = json.loads((root/'physics_train/gazebo_links.json').read_text())
    items = []
    for name, link in links.items():
        rotation = Rotation.from_quat(link['pose']['orientation']).as_matrix()
        inertia_rotation = rotation @ Rotation.from_quat(link['com']['orientation']).as_matrix()
        position = np.array(link['pose']['position']) + rotation @ link['com']['position']
        xx, yy, zz, xy, xz, yz = link['inertia']
        inertia = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
        if name.startswith('rotor_'):
            inertia = np.diag([(xx+yy)/2, (xx+yy)/2, zz])
        items.append((link['mass'], position, inertia_rotation @ inertia @ inertia_rotation.T))
    mass = sum(row[0] for row in items)
    com = sum(m*p for m, p, _ in items)/mass
    inertia = sum(i + m*((p-com)@(p-com)*np.eye(3)-np.outer(p-com, p-com)) for m, p, i in items)
    sdf = ET.parse(args.sdf)
    motors = [p for p in sdf.findall('.//plugin') if p.find('motorConstant') is not None]
    if len(motors) != 4:
        raise ValueError('Expected four motor plugins')
    names = dict(motor_constant='motorConstant', moment_constant='momentConstant',
                 time_constant_up_s='timeConstantUp', time_constant_down_s='timeConstantDown',
                 rotor_drag_coefficient='rotorDragCoefficient', rolling_moment_coefficient='rollingMomentCoefficient')
    constants = {}
    for key, name in names.items():
        values = [float(m.findtext(name)) for m in motors]
        if len(set(values)) != 1:
            raise ValueError('Motor constants are not identical: '+name)
        constants[key] = values[0]
    truth = dict(mass_kg=mass, com_flu_m=com.tolist(),
                 spin_averaged_aggregate_inertia_kg_m2=inertia.tolist(), **constants)
    (root/'gazebo_truth.json').write_text(json.dumps(truth, indent=2)+'\n')
    config = yaml.safe_load((root/'physics_train/config.yaml').read_text())
    config['mass_kg'] = mass
    config['rotor_positions_flu_m'] = [(np.array(links['rotor_'+str(i)]['pose']['position'])-com).tolist() for i in range(4)]
    (root/'physics_fit_config.yaml').write_text(yaml.safe_dump(config))
    if not (root/'physics_report.yaml').exists():
        print('Known inputs prepared; run physics fit, then repeat comparison.')
        return
    report = yaml.safe_load((root/'physics_report.yaml').read_text())
    rows = [('motor_constant', constants['motor_constant'], report['propulsion']['gazebo_motor_constant_n_per_rad_s_squared'])]
    rows += [(name, inertia[i, i], report['rotation']['aggregate_inertia_diagonal_kg_m2'][i]) for i, name in enumerate(('Ixx', 'Iyy', 'Izz'))]
    rows += [('moment_constant', constants['moment_constant'], report['rotation']['gazebo_moment_constant_m'])]
    rows = [dict(parameter=n, reference=t, identified=v, error_percent=100*(v/t-1)) for n,t,v in rows]
    with (root/'comparison.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    comparison = dict(dynamic_validation_passed=report['passed'], parameter_accuracy_passed=all(abs(r['error_percent'])<=10 for r in rows), reference_tolerance_percent=10, parameters=rows,
        effective_motor_tau_s=report['propulsion']['common_motor_time_constant_s'],
        actual_motor_tau_up_s=constants['time_constant_up_s'], actual_motor_tau_down_s=constants['time_constant_down_s'],
        notes=['Mass is known, not estimated.',
               'Inertia reference includes every link about aggregate COM, with spin-averaged rotor transverse inertia; it is not base_link inertia.',
               ('Measured RPM bypasses motor lag; no motor time constant was estimated.' if report.get('rotor_input')=='measured' else 'Single effective motor lag cannot equal separate rise/fall constants.'),
               'Fitted angular damping has different meaning/units from plugin rotor drag and rolling moment coefficients.',
               'Dynamic validation passing does not imply accurate parameters; reference errors are checked separately.'])
    (root/'comparison.json').write_text(json.dumps(comparison, indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.bar([r['parameter'] for r in rows], [r['identified']/r['reference'] for r in rows])
    ax.axhline(1, color='black', linestyle='--', label='Gazebo reference')
    ax.set_ylabel('Identified / reference'); ax.set_title('Parameter reference check: '+('within 10%' if comparison['parameter_accuracy_passed'] else 'outside 10%'))
    ax.legend(); fig.tight_layout(); fig.savefig(root/'comparison.png', dpi=160); plt.close(fig)
    print(json.dumps(comparison, indent=2))


if __name__ == '__main__':
    main()
