# CR-10S Depository

## Bed-Leveling

### Tutorials

- [Youtube: 6 steps to perfection](https://www.youtube.com/watch?v=zKpNxqWie_8)

### Macros

```gcode
G28         ; home xyz
G29 P1      ; run automated bed probing
G29 S1      ; save bed leveling mesh to slot 1
G29 F 10.0  ; set height correction fade to 10.0mm
G29 A       ; active bed leveling system
M500        ; save current setup
```

## Cura settings

### Start G-code

```gcode
M104 S210  ; Start warming extruder to 160
M190 S{material_bed_temperature_layer_0} ; Wait for Heat Bed temperature
G28        ; Home all axes
G29 A      ; Activate the UBL System.
G29 L1     ; Load UBL
G29 J2     ; 4-point level
G29 F10.0  ; Fade to 10mm
G92 E0     ; Reset Extruder

G1 Z2.0 F3000                   ; Move Z Axis up little to prevent scratching of Heat Bed
G1 X0.1 Y20 Z0.3 F5000.0        ; Move to start position
M109 S{material_print_temperature_layer_0} ; Wait for extruder temperature
G1 X0.1 Y200.0 Z0.3 F1500.0 E15 ; Draw the first line
G1 X0.4 Y200.0 Z0.3 F5000.0     ; Move to side a little
G1 X0.4 Y20 Z0.3 F1500.0 E30    ; Draw the second line
G92 E0                          ; Reset Extruder
G1 Z2.0 F3000                   ; Move Z Axis up little to prevent scratching of Heat Bed
G1 X5 Y20 Z0.3 F5000.0          ; Move over to prevent blob squish
```

### End G-code

```gcode
G91 ;Relative positioning
G1 E-2 F2700 ;Retract a bit
G1 E-2 Z0.2 F2400 ;Retract and raise Z
G1 X5 Y5 F3000 ;Wipe out
G1 Z10 ;Raise Z more
G90 ;Absolute positioning

G1 X0 Y{machine_depth} ;Present print
M106 S0 ;Turn-off fan
M104 S0 ;Turn-off hotend
M140 S0 ;Turn-off bed

M84 X Y E ;Disable all steppers but Z
```
