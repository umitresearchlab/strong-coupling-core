using namespace sc;

class JacobianElement {

    private:
        double m_data[6];

    public:

        JacobianElement();
        ~JacobianElement();

        void multiply(double * out, double * spatial, double * rotational);
};
