import sys.process._

Compile / compile := {
  // run the preprocess makefile as a dependency
  val makefileDir = baseDirectory.value / "src/main/resources/vsrc/cores"
  val exitCode = Process("make", makefileDir).!

  if (exitCode != 0) {
    sys.error(s"OFO verilog preprocessor makefile execution failed in ${makefileDir}")
  }

  (Compile / compile).value
}

