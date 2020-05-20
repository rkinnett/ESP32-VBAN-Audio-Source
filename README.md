# ESP32-VBAN-Audio-Source
Demonstration of audio sampling and streaming over UDP via VBAN protocol

Requires:  VBAN Voicemeeter running on a destination PC.
https://www.vb-audio.com/Voicemeeter/vban.htm

ESP32 samples audio via I2S, storing samples in Direct Memory Access (DMA) buffer.  After each batch of 256 samples (max allowed per VBAN protocol) are recorded, the ESP32 packages the samples into a UDP packet, per VBAN protocol.  The packet is transmitted to a destination IP address (which is hardcoded for this demo) where it may be unpacked by VBAN Receptor or (preferred) VBAN Voicemeeter.

This demo is part of a larger project which aims to use an ESP32 as a wireless audio relay between an FM VHF radio and a Windows PC to facilitate digital packet communication, with modem software running on the host PC to decode received audio and eventually to encode outgoing packets.

https://hackaday.io/project/170710-esp32-tnc-and-audio-relay-for-hfvhf-packet-radio
