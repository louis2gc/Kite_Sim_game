"""Run the standalone kite game."""
import argparse
from pathlib import Path
from dataclasses import replace
from kite_sim.config import DEFAULT_CONFIG,load_params


def main():
    parser=argparse.ArgumentParser(description='Kite: a keyboard-controlled physics playground')
    parser.add_argument('--config',default=DEFAULT_CONFIG,help='Physics YAML configuration')
    parser.add_argument('--turbulence',type=float,help='Relative gust standard deviation (e.g. 0.10)')
    parser.add_argument('--seed',type=int,help='Repeatable gust seed')
    parser.add_argument('--smoke',type=int,default=0,metavar='FRAMES',help='Headless test; render this many frames')
    parser.add_argument('--screenshot',help='Save the last frame as an image')
    args=parser.parse_args()
    if args.smoke<0:
        parser.error('--smoke must be nonnegative')
    params=load_params(args.config)
    if args.turbulence is not None:
        params=replace(params,turbulence_intensity=args.turbulence)
    if args.seed is not None:
        params=replace(params,wind_seed=args.seed)
    print(f'Launcher: {Path(__file__).resolve()}',flush=True)
    print(f'Configuration: {Path(args.config).resolve()}',flush=True)
    from kite_sim.app.game import run
    state=run(params,args.smoke,args.screenshot)
    # A headless test must fail visibly if the nominal flight exits its envelope.
    if args.smoke and state.stopped:
        raise SystemExit(1)


if __name__=='__main__':
    main()
