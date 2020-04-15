#ifndef PCBDECOMPACTION_EQUATION_H
#define PCBDECOMPACTION_EQUATION_H

///////////////////////
//    y-ax-b <=/>= 0
///////////////////////

enum class EqualType
{
    EQUAL;
    GEQ; //greater or equal to
    LEQ; //less than or equal to
};

class Equation
{
public:
    Equation(std::vector<double> &coeff, EqualType type) : m_coeff(coeff), m_type(type){};
    Equation(){};
    ~Equation(){};
    void print()
    {
        if (m_coeff.size() != 3)
            return;

        if (m_coeff[0] != 0)
            std::cout << m_coeff[0] << "y ";
        if (m_coeff[1] > 0)
            std::cout << "+ " << m_coeff[1] << "x ";
        else if (m_coeff[1] < 0)
            std::cout << m_coeff[1] << "x ";

        if (m_coeff[2] > 0)
            std::cout << "+ " << m_coeff[2];
        else if (m_coeff[2] < 0)
            std::cout << m_coeff[2];

        if (m_type == EqualType::EQUAL)
            std::cout << " = 0 " << std::endl;
        else if (m_type == EqualType::GEQ)
            std::cout << " >= 0 " << std::endl;
        else if (m_type == EqualType::LEQ)
            std::cout << " <= 0 " << std::endl;
    }

    double slope()
    {
        if (m_coeff[0] == 0)
            return 10000; //undefine slope
        return m_coeff[1] / m_coeff[0];
    }

private:
    std::vector<double> m_coeff;
    //    double m_slope;
    //    double m_intercept;
    EqualType m_type;
};

#endif