# Reproduction notes

Bench material for reproducing the SpineSense build: wiring and flashing procedures,
fault-finding entries, start-up acceptance checks, worked examples, and the design
records behind the addressing scheme.

The dissertation appendices state what the system is, what was measured, and where the
evidence stops; this file is the part you follow at the bench. Appendix section numbers
(A.6, A.11, A.12, B.2 ...) are cited where the two overlap.

## Address assignment: pseudocode and firmware entry points

```text
initialise the five TA0 lines and drive them all low
initialise I3C2
broadcast DISEC
broadcast RSTDAA

for i = 0 ... 4:
    raise TA0[i] only
    static_addr = 0x6B
    dynamic_addr = 0x32 + i
    send SETDASA(static_addr, dynamic_addr) directly
    wait for the target to be ready again
    drive TA0[i] low
    assert WHO_AM_I(dynamic_addr) == 0x73
```

The five-device connection test uses the following program:

`firmware/imu_i3c_connectivity_5/imu_i3c_connectivity_5.ino` (public repository)

The content hashes of the program and its build outputs are as follows:

| Artefact | SHA-256 |
| --- | --- |
| Source file `imu_i3c_connectivity_5.ino` | `5bcbab27f5a3a5c782433082f553623783eaedee79db61e90add7022c584db14` |
| `imu_i3c_connectivity_5.ino.bin` | `6aae7e1a3cf7def1dd10fb0d477c6fff999d926a868fb863d386cf0757b47163` |
| `imu_i3c_connectivity_5.ino.hex` | `1d4378d766a2752a695e6e3523d9bf5c33ce7648cc1f2e78b7a6dd5eabda6abc` |

The connection test of A.11 uses this program. The source file is released with the public repository, and the hash in the first row of the table above can be checked directly against the repository file. The `.bin` and `.hex` files are build outputs and are not released with the repository; an output rebuilt with the toolchain recorded in A.12 was not compared byte by byte with that binary. This program has no complete build manifest or rebuild proof of the kind used for the streaming firmware of A.12.

The frozen streaming firmware implements the same assignment sequence. The entry points in the connection test firmware are `resetDynamicAddresses()` and `setDynamicAddress()`, and the entry point in the streaming firmware is `assignDynamicAddresses()`:

`firmware/imu_i3c_xyz_sflp/imu_i3c_xyz_sflp.ino` (public repository)

The interrupt-driven HAL transfers use callbacks, a `volatile` completion flag and a timeout wait. Hardware synchronisation between the sensors was not measured here.

## Five-device connection test: procedure

1. Connect five STEVAL-MKI248KA assemblies directly to the NUCLEO-U385RG-Q with jumper wires of about 30 cm, bypassing the fan-out board, and check SCL, SDA, the five TA0 lines and the common ground one by one.
2. Build `imu_i3c_connectivity_5.ino` with STM32 Arduino core 2.12.0. The original connection test ran on Windows with Arduino IDE 2.3.8 and SWD flashing.
3. After flashing, open the ST-LINK virtual serial port at 115200 baud and reset the controller.
4. Check `RSTDAA: PASS`, then check that the five devices take 0x32–0x36 in turn.
5. Check that all five WHO_AM_I reads return 0x73; the final output should contain `SUMMARY: 5/5 IMUs connected` and `RESULT: ALL_PASS`.
6. Archive the complete serial output, the firmware version, photographs of the connection and the file hashes. The reported system results cite only records that correspond to this configuration.

## Five-device connection test: fault finding

| Visible symptom | Check first | Action |
| --- | --- | --- |
| No serial output | The virtual COM port, the PA9/PA10 configuration, the baud rate | Confirm that the ST-LINK virtual serial port is in use with the serial parameters set by the current firmware |
| I3C2 initialisation fails | PB13/PB14, the supply, the core and HAL build configuration | Check the pins and `build_opt.h` one by one; disconnect the targets first to rule out a short |
| RSTDAA fails | The SCL/SDA connections, the pull-ups and the common ground | Shorten the connections and check the levels; fit external pull-ups and measure again if necessary |
| Automatic enumeration finds only one device | Whether ENTDAA was used by mistake | Use the TA0 plus SETDASA procedure of this section instead |
| SETDASA fails on one device | The corresponding TA0 line, the GPIO mapping and the DIL24 contact | Check that TA0 line on its own, together with the response at static address 0x6B |
| The dynamic address is present but WHO_AM_I is not 0x73 | The address, the supply, CS and the device connections | Confirm that the target is ready again before reading, and check that CS is high |

The table lists the items in the order in which they should be checked; it does not state that each item is a confirmed fault cause.

## Frozen firmware: archive check and flashing

With the complete archive, check integrity with `MANIFEST.sha256` first and then flash the archived binary; with the public repository alone, rebuild the source under the toolchain and the board identifier of the table above. Rebuilding the source checks build reproducibility and does not have to be run before every acquisition. The archive does not store the exact flashing command, and that command was not replayed on hardware; it should be frozen at the same time as the complete start-up log is captured. The serial port should be opened before the reset and the start-up recorded verbatim, so that the earliest address and initialisation information is not lost.

## Frozen firmware: start-up acceptance sequence

The start-up acceptance sequence of the frozen firmware is:

1. the firmware title line and the five-interface mapping;
2. the five TA0 GPIOs initialise successfully;
3. I3C2 initialises successfully;
4. five SETDASA operations, with dynamic addresses 0x32–0x36 in order;
5. five WHO_AM_I reads all return 0x73;
6. the six-axis and SFLP configuration completes on all five devices;
7. the fourteen-field header is printed;
8. the first complete software frame containing five distinct interfaces is printed.

## Firmware control flow

```text
setup:
    open the 921600 baud serial port and print the interface map
    initialise TA0 and I3C2
    assign the dynamic addresses 0x32-0x36
    for each IMU:
        check WHO_AM_I
        configure the accelerometer and the gyroscope
        enable the SFLP quaternion output
    print the fourteen-field header

loop:
    wait for the next 8333 us software frame
    if late by more than four periods, re-anchor the schedule instant
    t_ms = the single software time label of this frame
    for IMU0 ... IMU4:
        on a six-axis read failure, emit the four-field READ_FAIL and move to the next device
        read the SFLP quaternion; on failure keep the six axes and write four NaN
        emit the fourteen-field record
```

## Coordinate processing: execution skeleton and reproduction checks

The pseudocode below is the execution skeleton of the coordinate processing function in the frozen implementation: `q_parent_common` and `q_child_common` correspond to step (a), `q_rel` to (b), `q_rel0` and `q_local` to (c), and `swing_twist` to (d). The skeleton stops at the two scalar read-outs; the extraction of $\theta_x$ and $\theta_y$ and the negation of the twist in B.2.4 are not expanded.

```text
for each accepted trial:
    load 14-field records and the trial-specific placement map
    retain complete timestamps containing all five sensor identities
    estimate each sensor's gravity tilt from the defined still window

    for each predefined parent-child pair:
        q_parent = normalize(SFLP quaternion of parent)
        q_child  = normalize(SFLP quaternion of child)

        q_parent_common = A_parent * q_parent * inverse(C_parent)
        q_child_common  = A_child  * q_child  * inverse(C_child)
        q_rel = inverse(q_parent_common) * q_child_common

        q_rel0 = quaternion_mean(q_rel within [-1.2, -0.2] s)
        q_local = inverse(q_rel0) * q_rel
        twist_deg, swing_deg = swing_twist(q_local, axis_u)

        save q_local, signed twist_deg, nonnegative swing_deg
```

The table below lists the checks that a reproduction should run as a minimum.

| Check | Expected result |
| --- | --- |
| Identity input | $q_{\mathrm{local}}=I$, with swing and twist both $0^\circ$ |
| Common left-multiplication | After the parent and the child are both left-multiplied by the same quaternion, $q_{\mathrm{rel}}$ is unchanged |
| Independent left-multiplication | When the parent and the child use different references, a non-zero residual should appear; otherwise the check is implemented incorrectly |
| Constant right-multiplication | After the reference end is right-multiplied by a constant quaternion, the three signed curves and the swing magnitude are all unchanged; after the moving end is right-multiplied by a constant quaternion about $\mathbf u$, $\theta_{\mathrm{tw}}$ and $\theta_{\mathrm{swing}}$ are unchanged and $(\theta_x,\theta_y)$ rotates as a whole within the plane by the corresponding angle (B.2.5) |
| Pure twist | $\theta_{\mathrm{twist}}$ equals the set angle and the swing is zero; after the negation of B.2.4, the output $\theta_{\mathrm{tw}}$ has the opposite sign to the set angle |
| Pure swing | The swing magnitude equals the set angle and the twist is close to zero |
| Double cover | $q$ and $-q$ denote the same rotation and the scalars are equivalent under circular distance; a different numerical representation is permitted at the $\pm180^\circ$ branch point |
| Pre-movement baseline window | A formal reproduction requirement should reject the bout when the window is missing, non-finite or insufficiently covered; the older fallback behaviour in the frozen classification artefact is given below |
| Axis definition | $\|\mathbf u\|=1$; the fixed axis and the data are expressed in the same coordinates |

## Coordinate processing: worked numerical example

This example uses constructed values to illustrate the three steps of taking the relative rotation, zeroing locally and reading out swing and twist, corresponding to B.2.2 to B.2.4; the values are set rather than measured. Let the mounting correction and the additional common-reference correction both be the identity, so that step (a) is the identity map, and let all rotations be about the common vertical axis:

$$
C_p^{-1}=C_c^{-1}=I,
\qquad
A_p=A_c=I,
$$

$$
q_z(\theta)
=
\left[
\cos\frac{\theta}{2},\,0,\,0,\,\sin\frac{\theta}{2}
\right].
$$

For a unit quaternion $q=[w,\mathbf v]$,

$$
q^{-1}=[w,-\mathbf v],
$$

and Hamilton multiplication is

$$
[w_1,\mathbf v_1]\otimes[w_2,\mathbf v_2]
=
\left[
w_1w_2-\mathbf v_1^\mathsf T\mathbf v_2,\,
w_1\mathbf v_2+w_2\mathbf v_1+\mathbf v_1\times\mathbf v_2
\right].
$$

The set orientations of the parent region and the child region are $0^\circ$ and $5^\circ$ at baseline and $10^\circ$ and $25^\circ$ during the movement; the two streams share one arbitrary reference of $30^\circ$, so the two outputs are $30^\circ$ and $35^\circ$ at baseline and $40^\circ$ and $55^\circ$ during the movement.

| Step | Operation | Result |
| --- | --- | --- |
| The two outputs at baseline | $\hat q_p(0)=q_z(30^\circ)$, $\hat q_c(0)=q_z(35^\circ)$ | $[0.9659,0,0,0.2588]$, $[0.9537,0,0,0.3007]$ |
| The baseline relationship | $[0.9659,0,0,-0.2588]\otimes[0.9537,0,0,0.3007]$ | $[0.9990,0,0,0.0436]=q_z(5^\circ)$ |
| The two outputs during the movement | $\hat q_p(t)=q_z(40^\circ)$, $\hat q_c(t)=q_z(55^\circ)$ | $[0.9397,0,0,0.3420]$, $[0.8870,0,0,0.4617]$ |
| The current relationship | $[0.9397,0,0,-0.3420]\otimes[0.8870,0,0,0.4617]$ | $[0.9914,0,0,0.1305]=q_z(15^\circ)$ |
| Local zeroing | $[0.9990,0,0,-0.0436]\otimes[0.9914,0,0,0.1305]$ | $[0.9962,0,0,0.0872]=q_z(10^\circ)$ |
| Decomposition about $\mathbf u=[0,0,1]^\mathsf T$ | The vector part of $q_{\mathrm{local}}$ contains only a $z$ component | $q_{\mathrm{twist}}=q_{\mathrm{local}}$, $q_{\mathrm{swing}}=[1,0,0,0]$ |
| Scalar read-out | $2\operatorname{atan2}(0.0872,\,0.9962)$ | $\theta_{\mathrm{twist}}=10^\circ$, $\theta_{\mathrm{swing}}=0^\circ$ |

The common $30^\circ$ reference cancels in the parent–child relative operation, the $5^\circ$ regional pair offset already present at baseline cancels in the local zeroing, and the $10^\circ$ regional relative change remains. The actual program performs the full quaternion multiplication and does not use angle subtraction in its place.

All rotations in the example above are about $\mathbf u$, so the swing part degenerates to the identity quaternion. If instead the parent region is stationary and the child region is rotated by $20^\circ$ about the $x$ axis, the result is a pure swing case:

| Step | Operation | Result |
| --- | --- | --- |
| Local motion | $q_{\mathrm{local}}=q_x(20^\circ)$ | $[0.9848,\,0.1736,\,0,\,0]$ |
| Decomposition about $\mathbf u=[0,0,1]^\mathsf T$ | $\mathbf u^\mathsf T\mathbf v=0$, so $\mathbf v_{\mathrm{twist}}=\mathbf 0$ | $q_{\mathrm{twist}}=[1,0,0,0]$, $q_{\mathrm{swing}}=q_{\mathrm{local}}$ |
| Scalar read-out | $\dfrac{360^\circ}{\pi}\arccos(0.9848)$; $2\operatorname{atan2}(0.1736,\,0.9848)$ | $\theta_{\mathrm{twist}}=0^\circ$, $\theta_{\mathrm{swing}}=20.0^\circ$, $\theta_x=20.0^\circ$, $\theta_y=0^\circ$ |

The swing rotation vector lies in the plane perpendicular to $\mathbf u$ (B.2.4), so the whole magnitude in this example is carried by $\theta_x$, and $\theta_y$ is zero. The two examples correspond respectively to the pure-twist and pure-swing rows of the reproduction check table in B.5.

## Design record: how the addressing scheme was arrived at

The candidate comparison in A.6 is the outcome of a process of elimination. This section records the criteria and the evidence used, and states why each route was excluded or adopted.

**Confirming the identifier collision.** An earlier bench test read the I3C identifiers of two ISM6HG256X devices directly, and the two instance identifiers were the same (`InstID = 0x09`). The field is written into OTP at the factory and the device provides no writable register, so ENTDAA arbitration cannot distinguish devices from the same batch and automatic enumeration discovers only one of them reliably. A survey of comparable six-axis I3C devices found no counter-example, and changing the device model does not remove this constraint.

**Change of route.** The early route was to power the sensors in sequence through high-side switches, so that only one device is present on the bus at any moment; that route needs additional switching devices and a board revision. An earlier technical discussion in the project moved the route to SETDASA: it selects the target by static address and does not read the factory identifier at all, so the identifier collision is no longer relevant. The TA0 pin itself sets the static address, and five GPIOs are enough to create the required uniqueness one device at a time, without new hardware. This study carried out the feasibility check, the register-level implementation and the five-device verification of this route, as the next two paragraphs describe.

**Evidence.** The internal pin status table (Table 25) of the datasheet (DS15034 Rev 2) gives two premises: SDO/TA0 defaults to a high-impedance input with no built-in pull-up, and the pull-up is controlled in software by the `SDO_PU_EN` bit of `PIN_CTRL (02h)`. The target address definition in Section 5.1.2 of the same datasheet specifies that the device responds to `0x6A` when the pin is tied to ground and to `0x6B` when it is tied to the supply voltage. This confirmed that a controller GPIO can drive the TA0 level reliably and switch the static address between the two. A review of the board routing further confirmed that the pin is not held at a fixed level.

**Implementation and verification.** The two register-level I3C frames, the CCC frame and the Private Write frame, were implemented after a field-by-field check, with an added power-up delay and a per-device status confirmation. Connectivity verification was completed on five commercial modules on 22 May 2026: the five devices took `0x32` to `0x36` in turn and the device identity register returned `0x73` for all of them. A.11 gives the complete serial log, the reproduction steps and the fault-finding notes.

## Evaluation: per-run clock mapping parameters

| Acquisition | Run | Movement blocks covered | $a$ | $b$ (s) | Synchronisation correlation | Estimation method |
| --- | --- | --- | ---: | ---: | ---: | --- |
| T02 | Main run | B1 to B5, B6a | 1.003500 | 11.20 | Not recorded | Manual synchronisation, carried over |
| T02 | Continuation run | B6b | 1.002750 | −1067.45 | Not recorded | Manual synchronisation, carried over |
| T03 | Single run | B1 to B6 | 1.003250 | 2.00 | Not recorded | Manual synchronisation, carried over |
| T04 | Single run | B1 to B6 | 1.003500 | 20.25 | 0.650 | Automatic envelope alignment |
| T05 | Single run | B1 to B6 | 1.003250 | −13.10 | 0.834 | Automatic envelope alignment |
| T06 | seg1 | B1, B2 | 1.002750 | −1.05 | 0.806 | Automatic envelope alignment |
| T06 | seg2 | B3 | 1.003500 | −745.00 | 0.843 | Automatic envelope alignment |
| T06 | seg3 | B4 to B6 | 1.002250 | −1036.50 | 0.938 | Automatic envelope alignment |
| T08 | Single run | B1 to B6 | 1.003250 | 10.35 | 0.927 | Automatic envelope alignment |
| T09 | Single run | B1 to B6 | 1.003250 | 9.00 | 0.877 | Automatic envelope alignment |
| T10 | seg1 | B1 to B3 | 1.002500 | 7.00 | 0.726 | Automatic envelope alignment |
| T10 | seg2 | B4 to B6 | 1.002500 | −823.85 | 0.878 | Automatic envelope alignment |
| T11 | Single run | B1 to B6 | 1.003250 | 7.40 | 0.946 | Automatic envelope alignment |
| T12 | Single run | B1 to B6 | 1.003250 | 8.10 | 0.955 | Automatic envelope alignment |
| T13 | Single run | B1 to B6 | 1.003250 | 20.65 | 0.821 | Automatic envelope alignment |
| T14 | Single run | B1 to B6 | 1.003500 | −3.05 | 0.880 | Automatic envelope alignment |
| T15 | Single run | B1 to B6 | 1.003500 | 4.10 | 0.866 | Automatic envelope alignment |

## Classification: full hyper-parameter grids

**Parameter grids.**

| Pipeline | Grid | Combinations |
|---|---|---:|
| Logistic regression | L2 penalty, $C\in\{0.01,0.1,1,10\}$; elastic net, $C\in\{0.03,0.3,3\}$ with $\ell_1$ ratio $\in\{0.25,0.50,0.75\}$ | 13 |
| Linear SVM | $C\in\{0.01,0.1,1,10\}$ | 4 |
| RBF-SVM | $C\in\{0.1,1,10,100\}$ with $\gamma$ multipliers $\in\{0.25,1,4\}$; the denominator of $\gamma$ is the number of features left after variance filtering on the training fold | 12 |
| Weighted LDA | Shrinkage strength $\in\{0.95,0.80,0.50,0.20,0.05\}$ | 5 |
| Constrained random forest | Tree depth $\in\{4,8,12\}$, minimum samples per leaf $\in\{15,5\}$, feature subsampling $\in\{\sqrt{p},0.30\}$; 1000 trees fixed | 12 |
| Constrained HGB | Tree depth $\in\{2,3\}$, L2 strength $\in\{10.0,1.0\}$, learning rate and iteration count paired as $(0.10,100)$, $(0.05,200)$, $(0.03,300)$; minimum samples per leaf fixed at 20, early stopping disabled | 12 |
| `Dummy` | No parameters; returns the most frequent class of the training set | 1 |

## Chapter 11 supplementary analyses: methods and outputs

This section records the methods and outputs of the six supplementary analyses cited in the Chapter 11 discussion. All six re-aggregate existing outputs and do not rerun the acquisition, the coordinate chain or the locked evaluation. Their conclusions are not reported in Chapter 10, because the methods of Chapter 6 do not declare these analyses. The executable implementation of three of the analyses is `tools/twist_bench/paper_supplements/ch10_supplement.py`.

**Replication check.** The supplementary analysis reimplements the thirteen-fold held-out-participant procedure and reproduces item by item the preprocessing chain of `locked_track_a/core.py` (median imputation, removal of zero-variance columns, standardisation), the hyperparameter set `logistic:00` selected in common across all thirteen folds ($C = 0.01$, L2 penalty, `lbfgs` solver), the inverse-participant weighting scheme, and the macro-F1 over the fixed six-class label set. The resulting participant-equal macro-F1 is 0.9544739, identical to the locked run in every digit; the predicted labels of the 1,387 bouts match one by one, and the 64 misclassified bouts coincide exactly. The analyses below therefore use the same predictions as Section 10.1.

**Structure of the misclassifications.** Amplitude is taken as the absolute peak of the concurrent optical reference; the standardisation within participant × movement class uses the median and standard deviation of that group and is intended to remove differences in individual range of movement and in the amplitude of each movement class. The quality grade comes from the block-level annotation carried in the feature table of the classification branch. It has four values and covers 79 movement blocks: 48 blocks `clean`, 13 blocks `limitation`, 9 blocks `not_clean` and 9 blocks `low_conf`. Here `limitation` refers specifically to the established limitation annotated uniformly on the extension blocks because their range of movement is too small, and all 13 blocks are B2; the other three values reflect quality decisions on the measurement side. This annotation differs in both denominator and grade names from the three grades of E.5, which are counted over the 74 convertible blocks; the two are not interchangeable. The column does not enter the classification pipelines as a feature (E.6.2 gives the blocked pattern for column names). The cross-branch comparison joins the block-level misclassification rate with the 77 within-block correlation coefficients of the block-level correlation table of that run, with a binary threshold at $r = 0.8$. The odds ratio and Fisher’s exact test count bouts, whereas the grouping variable $r$ is a block-level property and the bouts within one block are not independent; the resulting $p$ values are therefore biased downwards and are used only to describe the gap between the misclassification rates of the two groups. They do not constitute an inference. Robustness is checked by excluding participants one at a time, and the three variants reported in the main text (whole cohort, T09 excluded, T09 and T03 excluded) come from that sequence.

**Direction-feature diagnostic.** The flip count is the number of direction features whose median in one block has the opposite sign to the median over the remaining participants for the same movement class, with both medians above 0.1 in absolute value; it is counted block by block over the thirty direction features. The within-block comparison takes the difference between the median direction features of the misclassified and the correctly classified bouts of the same block.

**Test of amplitude invariance at the discrimination level.** The movement amplitude of each participant is measured by the median absolute peak of the concurrent optical reference. This external quantity is used instead of a feature of the system itself, so that the condition under test is not measured with the object under test. The two definitions, all movements and flexion, are computed separately: the first covers all six classes and the second corresponds to the single-class comparison of Figure C1(b). The correlation coefficients are the Spearman and Pearson correlations between the amplitude median of each participant and the six-class macro-F1 of that participant, $n = 13$.

**Selection stability (Section 10.1).** The hyperparameters selected in each fold come from the selection-stability table of the locked run. E.6.1 defines the simplicity ranking; the logistic regression family is ordered by penalty type and by ascending $C$.

**Load-bearing features.** The grouped permutation is run on the bouts of the held-out participant of each outer fold: the row order of a specified feature group is shuffled while the other columns are left unchanged, the fold is scored again, and the difference between the baseline score and the permuted score is taken. Each group in each fold is repeated 20 times and averaged, and the mean and standard deviation are then taken over the thirteen folds. The random number generator is initialised with the fixed seed 20260721 in each fold. The grouping has three levels: the ten regional pairs (13 columns each), the thirteen feature families (10 columns each), and two semantic blocks (a direction block of 60 columns, containing three signs and three normalised direction components; an amplitude block of 60 columns, containing three proportions, two log ratios and the twist dominance). The two blocks are 120 columns in total, and the 10 columns of the reversal-count family belong to neither. The three levels of grouping overlap, so they are not a variance decomposition; a negative point estimate means that the permutation of that group did not reduce the score on average, and at the scale of the between-participant standard deviation it is indistinguishable from zero.

| Output | Content |
| --- | --- |
| `tools/twist_bench/paper_supplements/ch10_supplement.py` | The executable implementation of all six analyses, including the replication-check assertions; released with the public repository |
| `ch10_supplement.json` | Summary results, covering the replication check, the structure of the misclassifications, the cross-branch comparison, the load-bearing features and the direction diagnostic |
| `feature_importance.csv` | Per-group permutation importance for the three levels of grouping (mean, standard deviation, per-fold extremes) |
| `bout_level_rows.csv` | Prediction, correctness, amplitude, standardised amplitude and quality grade of the 1,387 bouts |
| `block_level_join.csv` | Joined table of classification misclassification rate and within-block correlation for the 77 movement blocks |
| `direction_flips.csv` | Direction-feature flip count and misclassification rate for the 79 movement blocks |

> Table: Output list for the supplementary analyses of the classification branch. Apart from the implementation script in the first row, the other five items are participant-level outputs, written to disk with that run; an environment without these outputs cannot rerun the script directly.

## Design record: alternative architectures and engineering trade-offs

The final architecture uses one shared I3C bus and five dedicated TA0 lines. The table that maps each GPIO to a dynamic address fixes device identity. At start-up the reset must come before the devices are assigned one at a time; a wiring error on any TA0 line leaves the corresponding device without a correct identity.

| Engineering constraint | Route excluded | Final handling |
| --- | --- | --- |
| The five devices share one model and one static address | Direct I²C in parallel | Per-device selection through TA0, dynamic addresses assigned by SETDASA |
| Identifier collision within one production batch | ENTDAA automatic enumeration | No reliance on automatic enumeration |
| No external multiplexer added | An I²C multiplexer or a bridge controller | The I3C2 peripheral and the five GPIOs already on the controller |
| The Arduino layer has no I3C interface | Waiting for a higher-level library or changing platform | STM32 HAL I3C called from within the sketch |
| Garment routing may become longer later | Assuming the internal pull-ups hold at every length | The fan-out board keeps provision for external pull-ups and retains the obligation to measure |

This study completed the final pin topology, the HAL integration, the five-device program, the connection debugging and the verification of the working configuration. The capabilities of the devices themselves are not claimed as original work.

---

Source: appendices of the MSc dissertation *SpineSense: a five-IMU garment for trunk
motion monitoring* (UCL REAT, 2026). This material was moved out of the appendices
because it is operational rather than argument.
