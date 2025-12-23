### Problem:

Using scripts/insert-includes.py to combine Verilog files together can create a syntax bug in the resulting combined Verilog file. When a \`include is followed immediately by a module definition the resulting file concatenates the included verilog without a newline thus creating a endmodulemodule syntax bug, e.g.:

combined.v
```
endmodulemodule Top(...
```

This is invalid Verilog syntax and fails compilation.

### Cause:

The script writes the included file directly without adding newlines before and after. Below is an example of this.

Minimal example:

top.v:
```
`include "alu.v"
module Top(input [3:0] a,b, output [3:0] sum);
    ALU alu_inst(.a(a),.b(b),.sum(sum));
endmodule
```
alu.v:
```
module ALU(input [3:0] a,b, output [3:0] sum);
    assign sum = a + b;
endmodule
```

### Resulting combined.v:
```
module ALU(input [3:0] a,b, output [3:0] sum);
    assign sum = a + b;
endmodulemodule Top(input [3:0] a,b, output [3:0] sum);
    ALU alu_inst(.a(a),.b(b),.sum(sum));
endmodule
```

### Fix:

Modify insert-includes.py to add newlines before and after including files like so:
```
def process_helper(in_fname, out_f, inc_dirs, replaced_includes):
    ...
    for num, line in enumerate(lines, 1):
        ...
        out_f.write("\n")  # <-- add a newline before including
        process_helper(inc_file_name, out_f, inc_dirs, replaced_includes)
        out_f.write("\n")  # <-- add a newline after including
```

This ensures the merged combined.v is always syntactically valid.