"""Mode 1: installed Cyclo hand retargeting, with its original solver/filter.

Keep the existing VR input, hand command topics and outer HOLD gate. This
entrypoint deliberately delegates all landmark-to-joint processing to Cyclo.
"""


def main():
    from scripts.teleop_retargeting import main as cyclo_main

    print('[HAND MODE 1] Cyclo original retargeting; native solver/filter; '
          'no custom thumb correction', flush=True)
    cyclo_main()


if __name__ == '__main__':
    main()
