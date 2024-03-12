This module contains "declarative" experiment descriptions.

That is, anything within the `mgodpl::declarative` namespace is a description of an experiment.

To whatever extent possible, the data in this namespace should be POD (plain old data); it "is" rather than "does",
and can be easily serialized and deserialized (to/from JSON, for example).

Each of the structs in here can be "instantiated": that is, they can be fed to logic that will set up the environment
that they describe and run the experiment, ideally in a deterministic way, so these structs will likely contain
a seed for the random number generator.