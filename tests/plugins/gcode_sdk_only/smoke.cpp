/*
Run the isolated SDK DLL's firmware probe. The dynamic loader resolves its host
C ABI and SDK implementation before main() can report the probe's exit status.

    main()
    `-- gcode_sdk_probe(): construct and exercise a public-SDK firmware session
*/

// Return zero only when the DLL's derived firmware encoder executes correctly.
extern "C" int gcode_sdk_probe();
// Forward the fixture result as the CTest process status without masking errors.
int main();

int main() { return gcode_sdk_probe(); }
