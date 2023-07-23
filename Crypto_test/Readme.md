##Crypto lib setup
1. Download the library from st website.
   "https://www.st.com/en/embedded-software/x-cube-cryptolib.html#get-software"

2. Link the crypto lib to your STM32Cube project.
   a. For compilation stage
<p align="center">
  <img src="imgs\Link for compilation.png" alt="Link for compilation stage">
</p>
   b. For Linking stage
<p align="center">
  <img src="imgs\Link with Linking stage.png" alt="Link with linking stage">
</p>

3. Include "cmox_crypto.h" file.
4. Start call the cypto APIs in your application.