#ifndef __QUADPROG_PREDICTOR_CORRECTOR__
#define __QUADPROG_PREDICTOR_CORRECTOR__

# include <Eigen/Dense>
# include <boost/shared_ptr.hpp>

namespace solver
{

class QuadProgPc
{
protected:
  double m_alpha, m_mu_aff, m_eta, m_sigma, m_nc_inv, m_toll, m_mu, m_mu_init, m_l_toll_init;
  unsigned int m_trial, m_iter_max, m_nu, m_nc, m_iter;
  
  double m_square_rd_toll;
  double m_square_rs_toll;
  double m_mu_toll ;
  
  Eigen::MatrixXd m_Hbar, m_MF, m_H, m_h, m_A, m_At;
  Eigen::VectorXd m_rd, m_rs, m_rslm, m_rslp, m_vetm, m_vetp, m_D, m_sol, m_Msol, m_du, m_ds, m_dlm, m_dlp;
  
public:
  QuadProgPc(const Eigen::Ref<const Eigen::MatrixXd>& H,
  const Eigen::Ref<const Eigen::MatrixXd>& A)
  {
    m_H = H;
    m_nu = H.cols();
    m_A = A;
    m_nc = A.cols();
    m_At = A.transpose();
    m_nc_inv = 1.0/m_nc;
    
    m_iter_max = 200;
    m_square_rd_toll = 0.01;
    m_square_rs_toll = 1e-8;
    m_l_toll_init = 1e-8;
    m_mu_toll = 1e-5;
    m_mu_init = 1e-4;
    m_toll = 1e-12;
    
    m_Hbar.resize(m_nu, m_nu);
    m_MF.resize(m_nu, m_nu);
    
    m_D.resize(m_nc);
    m_sol.resize(m_nu);
    m_Msol.resize(m_nu);
    m_du.resize(m_nu);
    m_ds.resize(m_nc);
    m_dlm.resize(m_nc);
    m_dlp.resize(m_nc);
    
    m_rd.resize(m_nu);
    m_rs.resize(m_nc);
    m_rslp.resize(m_nc);
    m_rslm.resize(m_nc);
    m_vetm.resize(m_nu);
    m_vetp.resize(m_nu);
    
  };
  
  int solve(  const Eigen::Ref<const Eigen::VectorXd>& h, 
              const Eigen::Ref<const Eigen::VectorXd>& bm, 
              const Eigen::Ref<const Eigen::VectorXd>& bp, 
              Eigen::Ref<Eigen::VectorXd> u, 
              Eigen::Ref<Eigen::VectorXd> s, 
              Eigen::Ref<Eigen::VectorXd> lm, 
              Eigen::Ref<Eigen::VectorXd> lp)
  {
    m_iter = 0;
    
    m_du.setZero();
    
    for (unsigned int idx = 0;idx<m_nc;idx++)
    {
      if (s(idx) < (bm(idx)+m_mu_init))
        s(idx) = (bm(idx)+m_mu_init);
      else if (s(idx) > (bp(idx)-m_mu_init))
        s(idx) = (bp(idx)-m_mu_init);
      if (lm(idx) < m_l_toll_init)
        lm(idx) = m_l_toll_init;
      if (lp(idx) < m_l_toll_init)
        lp(idx) = m_l_toll_init;
    }
    m_mu = (lm.dot(s-bm)+lp.dot(bp-s)) * m_nc_inv;
    
    while (m_iter<m_iter_max)
    {  
      m_rd = m_H*u+h+m_A*(lp-lm);
      m_rs = s-m_At*u;
      m_rslm = (s-bm).cwiseProduct(lm);
      m_rslp = (bp-s).cwiseProduct(lp);
      
      if ( (m_rs.squaredNorm() < m_square_rs_toll) && (m_rd.squaredNorm() < m_square_rd_toll ) && (m_mu < m_mu_toll) )
        return m_iter;
      
      for (unsigned int idx = 0;idx<m_nc;idx++)
      {
        if (s(idx) < (bm(idx)+m_toll))
          s(idx) = (bm(idx)+m_toll);
        else if (s(idx) > (bp(idx)-m_toll))
          s(idx) = (bp(idx)-m_toll);
        if (lm(idx) < m_toll)
          lm(idx) = m_toll;
        if (lp(idx) < m_toll)
          lp(idx) = m_toll;
      }
      
      m_D = (lm.array()/(s-bm).array() + lp.array()/(bp-s).array());
      m_sol = m_rd-m_A*((  m_rs.cwiseProduct(lm) - m_rslm ).array()/(s-bm).array() + ( m_rs.cwiseProduct(lp) + m_rslp ).array()/(bp-s).array()).matrix();
      m_Hbar = m_H+m_A*(m_D.asDiagonal()*m_At);
      
      m_MF = m_Hbar.triangularView<Eigen::Lower>().solve((Eigen::MatrixXd)m_Hbar.diagonal().asDiagonal()-(Eigen::MatrixXd)m_Hbar.triangularView<Eigen::Upper>());
      m_Msol = m_Hbar.triangularView<Eigen::Lower>().solve(-m_sol);
      
      m_trial = 0;
      while (((m_Hbar*m_du+m_sol).squaredNorm() >1e-4) && m_trial<10)
      {
        m_trial++;
        for (int idx = 0;idx<10;idx++)
          m_du=m_MF*m_du+m_Msol;
      }
      
      m_ds = -m_rs+m_At*m_du;
      m_dlm = -(m_rslm+lm.cwiseProduct(m_ds)).array()/(s-bm).array();
      m_dlp = (lp.cwiseProduct(m_ds)-m_rslp).array()/(bp-s).array();
      
      m_alpha = 1;
      for (unsigned int idx = 0;idx<m_nc;idx++)
      {
        if (m_ds(idx) < 0)
          m_alpha = std::min(m_alpha, (bm(idx)-s(idx))/m_ds(idx));
        else if (m_ds(idx) > 0)
          m_alpha = std::min(m_alpha, (bp(idx)-s(idx))/m_ds(idx));
        if (m_dlm(idx) < 0)
          m_alpha = std::min(m_alpha, -lm(idx)/m_dlm(idx));
        if (m_dlp(idx) < 0)
          m_alpha = std::min(m_alpha, -lp(idx)/m_dlp(idx));  
      }
      
      m_mu_aff = ( (lm+m_alpha*m_dlm).dot(s+m_alpha*m_ds-bm) + (lp+m_alpha*m_dlp).dot(bp-s-m_alpha*m_ds) ) * m_nc_inv;
      m_sigma = pow(m_mu_aff/m_mu, 3);
      m_eta = std::max(0.9, std::min(0.999, 1-m_sigma));
      
      m_vetm = (pow(m_eta*m_alpha, 2.0) *m_ds.cwiseProduct(m_dlm)).array()-m_sigma*m_mu;
      m_vetp = (pow(m_eta*m_alpha, 2.0) *m_ds.cwiseProduct(m_dlp)).array()-m_sigma*m_mu;
      
      m_rslm.noalias() += m_vetm;
      m_rslp.noalias() += m_vetp;
      
      
      m_sol.noalias()-= m_A*((  - m_vetm ).array()/(s-bm).array() + ( m_vetp ).array()/(bp-s).array()).matrix();
      m_Msol = m_Hbar.triangularView<Eigen::Lower>().solve(-m_sol);
      
      m_trial = 0;
      while (((m_Hbar*m_du+m_sol).squaredNorm() >1e-4) && m_trial<10)
      {
        m_trial++;
        for (int idx = 0;idx<10;idx++)
          m_du=m_MF*m_du+m_Msol;
      }
      
  
      m_ds = -m_rs+m_At*m_du;
      m_dlm = -(m_rslm+lm.cwiseProduct(m_ds)).array()/(s-bm).array();
      m_dlp = (lp.cwiseProduct(m_ds)-m_rslp).array()/(bp-s).array();
      
      m_alpha = 1;
      for (unsigned int idx = 0;idx<m_nc;idx++)
      {
        if (m_ds(idx) < 0)
          m_alpha = std::min(m_alpha, (bm(idx)-s(idx))/m_ds(idx));
        else if (m_ds(idx) > 0)
          m_alpha = std::min(m_alpha, (bp(idx)-s(idx))/m_ds(idx));
        if (m_dlm(idx) < 0)
          m_alpha = std::min(m_alpha, -lm(idx)/m_dlm(idx));
        if (m_dlp(idx) < 0)
          m_alpha = std::min(m_alpha, -lp(idx)/m_dlp(idx));  
      }
      
      u.noalias()  += (m_alpha*m_eta)*m_du;
      lm.noalias() += (m_alpha*m_eta)*m_dlm;
      lp.noalias() += (m_alpha*m_eta)*m_dlp;
      s.noalias()  += (m_alpha*m_eta)*m_ds;
      m_mu = ( lm.dot(s-bm) + lp.dot(bp-s) ) * m_nc_inv;

      m_iter++;
    }
    
    return m_iter;
  }
};

inline int quadprog_pc(
  const Eigen::Ref<const Eigen::MatrixXd>& H,
  const Eigen::Ref<const Eigen::VectorXd>& h, 
  const Eigen::Ref<const Eigen::MatrixXd>& A, 
  const Eigen::Ref<const Eigen::MatrixXd>& At, 
  const Eigen::Ref<const Eigen::VectorXd>& bm, 
  const Eigen::Ref<const Eigen::VectorXd>& bp, 
  Eigen::Ref<Eigen::VectorXd> u, 
  Eigen::Ref<Eigen::VectorXd> s, 
  Eigen::Ref<Eigen::VectorXd> lm, 
  Eigen::Ref<Eigen::VectorXd> lp, 
  const unsigned int& nu, 
  const unsigned int& nc, 
  const double& nc_inv, 
  const double& toll, 
  double& mu
 )
{
  unsigned int iter = 0;
  double alpha, mu_aff, eta, sigma;
  int trial;
  
  unsigned int iter_max = 200;
  double square_rd_toll = 0.01;
  double square_rs_toll = 1e-8;
  double mu_toll = 1e-5;
  
  // TODO RIMUOVERE STATIC!!!!
  Eigen::MatrixXd Hbar, MF;
  Eigen::VectorXd rd, rs, rslm, rslp, vetm, vetp, D, sol, Msol, du, ds, dlm, dlp;
  Eigen::VectorXd solution(nu);
  solution.setZero();
  
  for (unsigned int idx = 0;idx<nc;idx++)
    if (s(idx) < (bm(idx)+mu))
      s(idx) = (bm(idx)+mu);
    else if (s(idx) > (bp(idx)-mu))
      s(idx) = (bp(idx)-mu);
  
  mu = (lm.dot(s-bm)+lp.dot(bp-s)) * nc_inv;
  
  
  while (iter<iter_max)
  {  
    rd = H*u+h+A*(lp-lm);
    rs = s-At*u;
    rslm = (s-bm).cwiseProduct(lm);
    rslp = (bp-s).cwiseProduct(lp);
    
    if ( (rs.squaredNorm() < square_rs_toll) && (rd.squaredNorm() < square_rd_toll ) && (mu < mu_toll) )
      return iter;
    
    for (unsigned int idx = 0;idx<nc;idx++)
      if (s(idx) < (bm(idx)+toll))
        s(idx) = (bm(idx)+toll);
      else if (s(idx) > (bp(idx)-toll))
        s(idx) = (bp(idx)-toll);
    
    D = (lm.array()/(s-bm).array() + lp.array()/(bp-s).array());
    sol = rd-A*((  rs.cwiseProduct(lm) - rslm ).array()/(s-bm).array() + ( rs.cwiseProduct(lp) + rslp ).array()/(bp-s).array()).matrix();
    Hbar = H+A*D.asDiagonal() *At;
    
    MF = Hbar.triangularView<Eigen::Lower>().solve((Eigen::MatrixXd)Hbar.diagonal().asDiagonal()-(Eigen::MatrixXd)Hbar.triangularView<Eigen::Upper>());
    Msol = Hbar.triangularView<Eigen::Lower>().solve(sol);
    
    trial = 0;
    while (((Hbar*solution-sol).squaredNorm() >1e-4) && trial<10)
    {
      trial++;
      for (int idx = 0;idx<10;idx++)
        solution=MF*solution+Msol;
    }
    
    du = -solution;
    ds = -rs+At*du;
    dlm = -(rslm+lm.cwiseProduct(ds)).array()/(s-bm).array();
    dlp = (lp.cwiseProduct(ds)-rslp).array()/(bp-s).array();
    
    alpha = 1;
    for (unsigned int idx = 0;idx<nc;idx++)
    {
      if (ds(idx) < 0)
        alpha = std::min(alpha, (bm(idx)-s(idx))/ds(idx));
      else if (ds(idx) > 0)
        alpha = std::min(alpha, (bp(idx)-s(idx))/ds(idx));
      if (dlm(idx) < 0)
        alpha = std::min(alpha, -lm(idx)/dlm(idx));
      if (dlp(idx) < 0)
        alpha = std::min(alpha, -lp(idx)/dlp(idx));  
    }
    
    mu_aff = ( (lm+alpha*dlm).dot(s+alpha*ds-bm) + (lp+alpha*dlp).dot(bp-s-alpha*ds) ) * nc_inv;
    sigma = pow(mu_aff/mu, 3);
    eta = std::max(0.5, std::min(0.999, 1-sigma));
    
    vetm = (pow(eta*alpha, 2.0) *ds.cwiseProduct(dlm)).array()-sigma*mu;
    vetp = (pow(eta*alpha, 2.0) *ds.cwiseProduct(dlp)).array()-sigma*mu;
    
    rslm.noalias() += vetm;
    rslp.noalias() += vetp;
    
    
    sol = rd-A*((  rs.cwiseProduct(lm) - rslm ).array()/(s-bm).array() + ( rs.cwiseProduct(lp) + rslp ).array()/(bp-s).array()).matrix();
    Msol = Hbar.triangularView<Eigen::Lower>().solve(sol);
    
    trial = 0;
    while (((Hbar*solution-sol).squaredNorm() >1e-4) && trial<10)
    {
      trial++;
      for (int idx = 0;idx<10;idx++)
        solution=MF*solution+Msol;
    }
    

    du = -solution;
    ds = -rs+At*du;
    dlm = -(rslm+lm.cwiseProduct(ds)).array()/(s-bm).array();
    dlp = (lp.cwiseProduct(ds)-rslp).array()/(bp-s).array();
    

    alpha = 1;
    for (unsigned int idx = 0;idx<nc;idx++)
    {
      if (ds(idx) < 0)
        alpha = std::min(alpha, (bm(idx)-s(idx))/ds(idx));
      else if (ds(idx) > 0)
        alpha = std::min(alpha, (bp(idx)-s(idx))/ds(idx));
      if (dlm(idx) < 0)
        alpha = std::min(alpha, -lm(idx)/dlm(idx));
      if (dlp(idx) < 0)
        alpha = std::min(alpha, -lp(idx)/dlp(idx));  
    }
    
    u.noalias()  += alpha*eta*du;
    lm.noalias() += alpha*eta*dlm;
    lp.noalias() += +alpha*eta*dlp;
    s.noalias()  += alpha*eta*ds;
    mu = ( lm.dot(s-bm) + lp.dot(bp-s) ) * nc_inv;

    iter++;
  }
  
  return iter;
};
  
  typedef boost::shared_ptr<solver::QuadProgPc> QuadProgPcPtr;
}


#endif