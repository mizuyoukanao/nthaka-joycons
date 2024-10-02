# ORCA GC Controller

- [yatsuna827/Orca-GC-Controller](https://github.com/yatsuna827/Orca-GC-Controller)
- [mizuyoukanao/Bluewhale](https://github.com/mizuyoukanao/Bluewhale)

| ORCA  | Nthaka |
| :---: | :----: |
|   Y   |   Y    |
|   B   |   B    |
|   A   |   A    |
|   X   |   X    |
|   L   |   L    |
|   R   |   R    |
|   Z   |   ZR   |
| Start |   +    |
| Reset |  Home  |

```mermaid
stateDiagram-v2
    s1 : {0x80}
    s2 : {0x80, 0x00}

    [*] --> s1 : 0x80
    [*] --> [*] : '@'
    s1 --> s1 : 0xFF
    s1 --> s2 : *
    s2 --> s2 : 0xFF
    s2 --> [*] : *
```