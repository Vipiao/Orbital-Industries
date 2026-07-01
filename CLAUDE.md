Use modern OpenGL.
C++ 20

Never use "using".
Always use double precision. So dvec3 and dquat and double...
Use uniform initialization.
Suggest refactors when reasonable before implementing new features.
Use m_ prefix for member variables and s_ for static.

Follow principles:
Single responsibility
Segregation of interface and implementation
Non cyclic dependencies (hirearchical or layer based ect)
- A dependency can be hard (#include or forward declaration) or soft (module A contains and index into module B array). If A depends on B, then B defines the interface for communication, A initiates communication. B is an exportable tool.
Reproducibility
- Should be deterministic (no raw pointers used for hashing that could influence iteration order)
Limit the size of a module to below ~1000 lines of code.

Limit line width to 100 columns.