# audioreact

Light LEDs in sync with the music! For Arduino Nano R2040. 

# Example

https://user-images.githubusercontent.com/18037091/182042012-71385c50-7a80-4f0d-832e-c5abcbd40ae2.mp4

# Hardware



# How it works
### Transient
In music the **sense of beat and rythmn** is generaly transported over **transients**. These are **shudden changes in loudess**.
For example: if a drummer is hitting a drum, the noise that is **purely produced by the mallet** hitting the drumhead would be the transient. 

See the waveform of a drum sound here, with the transient marked in red.
![grafik](https://user-images.githubusercontent.com/18037091/182035915-d8842836-a16a-44f6-ace0-7423b2aa3e1d.png)
### Loudness
There are **many ways** to calculate the loudness of an audio signal. For simplicity, I'm calculating the **difference between the most negative and the most positive sample** within the audio of the microphone.

See the same waveform with the loud part in red, rather quiet part in yellow and the quiet part in green.
![grafik](https://user-images.githubusercontent.com/18037091/182037206-de32467c-79b2-4eea-98cf-f47c104c2a08.png)

## Transient detection

If we simply say anything louder than threshold X, we would find that the LED **would just flicker**. This is because we're lookin looking **"too close"** at the audio signal. To fix this we simply **smoothout** this signal. 
Since I'm a game-programmer by trade, I'm using a **damped-spring** system to do this. (If you're familiar with the Unity Game-Engine, you propably know this function by the name of ``.SmoothDamp()``.) 

The flickering is gone, but we now find ourselfs **in need to constantly adjust** the theshold. We simply solve this **tracking the loudness a second time**, but with even more smoothing. We use this "ambient" loudness as a baseline. If the short time loudness **is bigger** than the ambient loudness, we detected a **transient** and light the LED.

Here is an rough visualisation of both curves. In blue the ambient loudness and in orange the short-term loudness.
![grafik](https://user-images.githubusercontent.com/18037091/182041555-982cbc05-5b26-4eac-93b3-5d85ec862171.png)


# Third-Party-Code
Uses Damped-Spring by Ryan Juckett: https://www.ryanjuckett.com/damped-springs/
