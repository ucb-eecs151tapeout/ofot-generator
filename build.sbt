import sys.process._

Compile / compile := {
  // Read RUN_TYPE from environment variables, defaulting to "pd"
  val runTypeFlag = sys.env.getOrElse("RUN_TYPE_FLAG", "pd")

  // Run the preprocess makefile as a dependency
  val makefileDir = baseDirectory.value / "src/main/resources/vsrc/cores"

  // Set RUN_TYPE as an argument if Makefile can access it
  //val exitCode = Process("make RUN_TYPE=" + runTypeFlag, makefileDir).!

  //if (exitCode != 0) {
  //  sys.error(s"OFO Verilog preprocessor Makefile execution failed in ${makefileDir}")
  //}

  // Call the original compile task after the preprocessor step
  (Compile / compile).value
}