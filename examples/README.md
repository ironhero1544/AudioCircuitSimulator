# Audio Circuit Simulator examples

## Reference project

`CompleteStereoVolume.acproj` remains the complete reference circuit. It has a
user-wired 16-bit stereo DAC bus, symmetric reconstruction filters, 10 kOhm
volume controls, DAC-midscale blocking capacitors, a true +41 V / -41 V supply,
DC-blocked and emitter-stabilized Class-AB output stages, and a 2 x 100 W /
8 Ohm speaker load.

## Fault projects

Every fault project starts from the reference circuit and keeps a complete
DAC-to-amplifier-to-speaker route. These are component/value failures rather
than trivial broken wires, so simulation stays audible and the analyzer shows
a distinct signature.

- `Fault01_BitWeightDAC.acproj`: the resistor ladder remains precise, but
  adjacent high-order bus lines are physically exchanged. Expect
  non-monotonic code steps and extreme harmonic distortion without the fault
  merely behaving like a volume control.
- `Fault02_StereoFilterChaos.acproj`: both volume controls and power stages are
  identical, but the left reconstruction LPF rolls off around 3.39 kHz while the
  right cutoff is around 159 kHz. This isolates EQ from channel volume, and the
  response-delta graph should become extremely large above the bass range.
- `Fault03_BrownoutClipper.acproj`: the 100 W amplifier is powered by undersized
  +18 V / -18 V, 1 A rails. The route stays valid, but loud peaks hit voltage
  and current limits and flatten visibly in the live waveform.
- `Fault04_ThermalDcHazard.acproj`: the large output capacitors are visibly
  present but electrically bypassed. Driver offset therefore reaches the
  speaker coil as DC and produces the DC/heating diagnosis while audio remains
  connected.
- `Fault05_CrossoverBandwidth.acproj`: the bias network remains stable, while
  the two voltage drivers have extremely low GBW and slew rate. Expect strong
  high-frequency and transient distortion rather than a Newton failure or
  total silence.

## Effect and response-profile projects

- `Effect06_ExtremePitchClock.acproj`: runs the I2S DAC at 6.144 MHz instead of
  the nominal 3.072 MHz BCLK. The engine turns the x2 clock mismatch into a
  real asynchronous sample-clock conversion: playback speed and pitch rise
  together without inventing extra amplifier gain.
- `Profile07_ATH_M50x.acproj`: applies an editable four-band transducer response
  approximating the ATH-M50x: a 150 Hz bass hump, 425 Hz recession, and upper
  presence/treble peaks. The load also uses the published 38 Ohm impedance and
  1.6 W maximum input value. Its amplifier is also converted to a low-gain,
  +/-12 V headphone driver instead of retaining the 100 W speaker drive. It is a
  representative voicing profile, not a
  claim that every physical unit, pad, or seal measures identically.

Load any `.acproj`, start simulation, then compare **LIVE PCM**, **STEREO
SCOPE**, and **CIRCUIT RESPONSE**. Component labels beginning with `FAULT:`
identify the intentionally incorrect parts without giving the repair value.

Run `tools/generate_fault_examples.py` after changing the reference layout to
regenerate all unpacked fault directories and packages.
