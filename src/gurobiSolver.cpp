/* Copyright 2019, Gurobi Optimization, LLC */

/* This example reads an LP model from a file and solves it.
   If the model is infeasible or unbounded, the example turns off
   presolve and solves the model again. If the model is infeasible,
   the example computes an Irreducible Inconsistent Subsystem (IIS),
   and writes it to a file */

#include "gurobiSolver.h"

bool GurobiSolver::solver(string lpFileName, string solFileName) {
    /*if (argc < 2) {
    cout << "Usage: lp_c++ filename" << endl;
    return 1;
  }*/

    try {
        GRBEnv env = GRBEnv();
        GRBModel model = GRBModel(env, lpFileName);

        model.optimize();

        int optimstatus = model.get(GRB_IntAttr_Status);

        if (optimstatus == GRB_INF_OR_UNBD) {
            model.set(GRB_IntParam_Presolve, 0);
            model.optimize();
            optimstatus = model.get(GRB_IntAttr_Status);
        }

        if (optimstatus == GRB_OPTIMAL) {
            double objval = model.get(GRB_DoubleAttr_ObjVal);
            cout << "Optimal objective: " << objval << endl;

            ofstream file;

            file.open(solFileName);
            GRBVar *fvars = 0;
            fvars = model.getVars();
            int cols = model.get(GRB_IntAttr_NumVars);
            for (int j = 0; j < cols; j++) {
                GRBVar v = fvars[j];
                file << v.get(GRB_StringAttr_VarName) << " "
                     << v.get(GRB_DoubleAttr_X) << endl;
            }
            file.close();

        } else if (optimstatus == GRB_INFEASIBLE) {
            cout << "Model is infeasible" << endl;

            // compute and write out IIS

            model.computeIIS();
            model.write("model.ilp");
        } else if (optimstatus == GRB_UNBOUNDED) {
            cout << "Model is unbounded" << endl;
        } else {
            cout << "Optimization was stopped with status = "
                 << optimstatus << endl;
        }

    } catch (GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch (...) {
        cout << "Error during optimization" << endl;
    }

    return 0;
}
