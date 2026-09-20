CPP=g++
CARGS=-std=c++17 -O3 -g0 -m64
BRKGAINC=-I ../nsbrkga/nsbrkga
INC=-I src $(BRKGAINC)
MKDIR=mkdir -p
RM=rm -rf
SRC=$(PWD)/src
BIN=$(PWD)/bin

clean:
	@echo "--> Cleaning compiled..."
	$(RM) $(BIN)
	@echo

$(BIN)/%.o: $(SRC)/%.cpp
	@echo "--> Compiling $<..."
	$(MKDIR) $(@D)
	$(CPP) $(CARGS) -c $< -o $@ $(INC)
	@echo

$(BIN)/test/instance_test : $(BIN)/instance/instance.o \
                            $(BIN)/test/instance_test.o
	@echo "--> Linking objects..."
	$(CPP) -o $@ $^ $(CARGS) $(INC)
	@echo
	@echo "--> Running test..."
	$(BIN)/test/instance_test
	@echo

instance_test : $(BIN)/test/instance_test

$(BIN)/test/solution_test : $(BIN)/instance/instance.o \
                            $(BIN)/solution/solution.o \
                            $(BIN)/test/solution_test.o
	@echo "--> Linking objects..."
	$(CPP) -o $@ $^ $(CARGS) $(INC)
	@echo
	@echo "--> Running test..."
	$(BIN)/test/solution_test
	@echo

solution_test : $(BIN)/test/solution_test

$(BIN)/test/nsbrkga_solver_test : $(BIN)/instance/instance.o \
                                  $(BIN)/solution/solution.o \
                                  $(BIN)/solver/solver.o \
                                  $(BIN)/solver/nsbrkga/decoder.o \
                                  $(BIN)/solver/nsbrkga/nsbrkga_solver.o \
                                  $(BIN)/test/nsbrkga_solver_test.o
	@echo "--> Linking objects..."
	$(CPP) -o $@ $^ $(CARGS) $(INC)
	@echo
	@echo "--> Running test..."
	$(BIN)/test/nsbrkga_solver_test
	@echo

nsbrkga_solver_test : $(BIN)/test/nsbrkga_solver_test

$(BIN)/exec/nsbrkga_solver_exec : $(BIN)/instance/instance.o \
                                  $(BIN)/solution/solution.o \
                                  $(BIN)/solver/solver.o \
                                  $(BIN)/solver/nsbrkga/decoder.o \
                                  $(BIN)/solver/nsbrkga/nsbrkga_solver.o \
                                  $(BIN)/utils/argument_parser.o \
                                  $(BIN)/exec/nsbrkga_solver_exec.o
	@echo "--> Linking objects..."
	$(CPP) -o $@ $^ $(CARGS) $(INC)
	@echo

nsbrkga_solver_exec : $(BIN)/exec/nsbrkga_solver_exec

tests : instance_test \
        solution_test \
        nsbrkga_solver_test

execs : nsbrkga_solver_exec

all : tests execs
