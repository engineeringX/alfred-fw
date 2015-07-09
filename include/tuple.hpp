#ifndef ALFRED_TUPLE
#define ALFRED_TUPLE

template<class U1, class U2>
class Pair {
public:
  Pair(U1 u1, U2 u2)
    : m_u1(u1)
    , m_u2(u2)
  {
  };
  ~Pair() {};

  U1 first() const { return m_u1; }
  U2 second() const { return m_u2; }

private:
  U1 m_u1;
  U2 m_u2;
};

template<class U1, class U2, class U3>
class Triple {
public:
  Triple(U1 u1, U2 u2, U3 u3)
    : m_u1(u1)
    , m_u2(u2)
    , m_u3(u3)
  {
  };
  ~Triple() {};

  U1 first() const { return m_u1; }
  U2 second() const { return m_u2; }
  U3 third() const { return m_u3; }

private:
  U1 m_u1;
  U2 m_u2;
  U3 m_u3;
};

#endif //ALFRED_TUPLE

