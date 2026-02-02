# Certificates

## Getting the Root CA certificate from the HiveMQ Broker
1. Ensure that you have `openssl`.
2. Launch `cmd` or a terminal.
3. Run the following:
```bash
openssl s_client -showcerts -connect fe26426fbe64463790fc2792777c8189.s1.eu.hivemq.cloud:8883
```
4. In the output, copy everything including `-----BEGIN CERTIFICATE-----`, up to AND including `-----END CERTIFICATE-----` that is under something that looks like this: 
```text
 1 s:C = US, O = Let's Encrypt, CN = ...
   i:C = US, O = Internet Security Research Group, CN = ISRG Root X1 

```
For HIVEMQ, ISRG is the provider of the Root CA.

5. Everything copied now is what `const char* root_ca` should be. Continue following the steps in the README to set up TLS.