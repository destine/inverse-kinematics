#ifndef __incl_body__
#define __incl_body__

class Body
{
    float m_length;

public:
    Body(float length)
    {
        m_length = length;
    }

    float getLength(void) const
    {
        return m_length;
    }
};

#endif
