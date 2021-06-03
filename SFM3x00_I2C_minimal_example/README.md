# SFM3x00 Minimal Example

## Configuration

Please note that the model selection is done manually, line 55 of [SFM3x00_minimal_example.ino](SFM3x00_minimal_example.ino#55)

```c++
54: // ACTION: select your component here from the enum above:
55: const uint8_t MODEL = SFM3200;
```

The available sensors can be found on line 36 of [SFM3x00_minimal_example.ino](SFM3x00_minimal_example.ino#36)

```c++
35: // supported sensors
36: enum SFM_MODEL {
37:   SFM3000 = 0,
38:   SFM3200,
39:   SFM3300,
40:   SFM3400,
41:   SFM_MODEL_LENGTH //< Note: this is not a valid value for 'MODEL' below
42: };
```