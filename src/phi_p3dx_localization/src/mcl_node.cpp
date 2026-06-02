#include "phi_p3dx_localization/localization_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <vector>
#include <numeric>
#include <random>
#include <algorithm>

// parametros do modelo de ruido de odometria
double a1 = 0.05;
double a2 = 0.01;
double a3 = 0.05;
double a4 = 0.01;

// variancia do modelo de observacao
double var_obs = 0.5;

// pula de 10 em 10 leituras do laser
int passo_laser = 10;

// celulas com valor >= 50 no mapa são consideradas ocupadas
int lim_livre = 50;

// Alcance max do laser no ray casting
float max_r = 8.0f;

class MCLNode : public LocalizationNode
{
public:
  MCLNode() : LocalizationNode("mcl_node")
  {
  }

private:

  double dist_acum = 100.0;  // inicia alto para forçar primeira atualização
  double ang_acum = 100.0;

  void update_particles() override
  {
    if (!map_received_ || !current_map_) 
      return;

    // acumula o movimento desde a última atualização
    dist_acum += u_t.trans;
    ang_acum += std::abs(u_t.rot1) + std::abs(u_t.rot2);

    // só atualiza se o robô andou pelo menos 5cm ou girou um pouco
    if (dist_acum < 0.05 && ang_acum < 0.05) 
      return;

    dist_acum = 0.0;
    ang_acum = 0.0;

    //SAMPLING — propaga as partículas com ruído de odometria
    sample_particles();

    //WEIGHTING e RESAMPLING — só se tiver leitura do laser
    if (z_t) {
      weight_particles();
      resample_particles();
    }
  }

  // SAMPLING
  // propaga cada partícula usando o modelo de movimento 
  // cada partícula recebe um ruído diferente sorteado
  void sample_particles()
  {
    double r1 = u_t.rot1;
    double tr = u_t.trans;
    double r2 = u_t.rot2;

    double dev1 = std::sqrt(a1 * r1 * r1 + a2 * tr * tr);
    double dev2 = std::sqrt(a3 * tr * tr + a4 * (r1 * r1 + r2 * r2));
    double dev3 = std::sqrt(a1 * r2 * r2 + a2 * tr * tr);

    // evita desvio zero (causaria erro na distribuição normal)
    if (dev1 < 1e-9) dev1 = 1e-9;
    if (dev2 < 1e-9) dev2 = 1e-9;
    if (dev3 < 1e-9) dev3 = 1e-9;

    // cria as 3 distribuições normais de média zero
    std::normal_distribution<double> dist1(0.0, dev1);
    std::normal_distribution<double> dist2(0.0, dev2);
    std::normal_distribution<double> dist3(0.0, dev3);

    for (size_t i = 0; i < particles_.size(); i++) {
      // amostra o ruído e soma ao movimento
      double chapeu_r1 = r1 + dist1(rng_);
      double chapeu_tr = tr + dist2(rng_);
      double chapeu_r2 = r2 + dist3(rng_);

      // atualiza pose da partícula com o ruido
      particles_[i].x += chapeu_tr * std::cos(particles_[i].theta + chapeu_r1);
      particles_[i].y += chapeu_tr * std::sin(particles_[i].theta + chapeu_r1);
      particles_[i].theta += chapeu_r1 + chapeu_r2;

      while (particles_[i].theta >  M_PI) 
        particles_[i].theta -= 2.0 * M_PI;
      while (particles_[i].theta < -M_PI) 
        particles_[i].theta += 2.0 * M_PI;
    }
  }

  // IMPORTANCE WEIGHTING
  // compara as leituras do laser com o ray casting
  void weight_particles()
  {
    double res = current_map_->info.resolution;
    double ox = current_map_->info.origin.position.x;
    double oy = current_map_->info.origin.position.y;
    int w = current_map_->info.width;
    int h = current_map_->info.height;

    float amin = z_t->angle_min;
    float ainc = z_t->angle_increment;
    int n = z_t->ranges.size();

    double soma = 0.0;

    for (size_t i = 0; i < particles_.size(); i++) {
      // converte posição da partícula para célula do mapa
      int c = (particles_[i].x - ox) / res;
      int r = (particles_[i].y - oy) / res;

      // verifica se a partícula não cai sobre alguma célula não livre
      bool bateu = false;
      if (c < 0 || c >= w || r < 0 || r >= h) {
        bateu = true;
      } else {
        int idx = r * w + c;
        if (idx >= 0 && idx < (int)current_map_->data.size()) {
          int8_t celula_raw = current_map_->data[idx];
          int celula = static_cast<int>(celula_raw);
          if (celula >= lim_livre) {
            bateu = true;
          }
        }
      }

      if (bateu) {
        particles_[i].weight = 0.0;
        continue;
      }

      // calcula p(z_t | x_t, m)
      double soma_pk = 0.0;
      int lidos = 0;

      // pula de 10 em 10 
      for (int k = 0; k < n; k += passo_laser) {
        float z = z_t->ranges[k];

        // Ignora leituras inválidas 
        if (!std::isfinite(z) || z <= z_t->range_min || z >= z_t->range_max) {
          continue;
        }

        if (z > max_r) {
          z = max_r;
        }
      
        double angulo = particles_[i].theta + amin + k * ainc;

        // simula o laser via ray casting
        float z_esp = ray_cast(particles_[i].x, particles_[i].y, angulo, max_r, res, ox, oy, w, h);
        double erro = z - z_esp;
        // gaussiana
        double p_k = std::exp(-0.5 * erro * erro / var_obs);

        soma_pk += p_k;
        lidos++;
      }

      if (lidos == 0) {
        particles_[i].weight = 0.0;
      } else {
        // usar a média das probabilidades evita underflow 
        particles_[i].weight = soma_pk / lidos;
      }

      soma += particles_[i].weight;
    }

    // normaliza os pesos para que a soma dê 1
    // Se a soma for zero, deixa todas com peso igual
    if (soma < 1e-300) {
      double peso_igual = 1.0 / particles_.size();
      for (size_t i = 0; i < particles_.size(); i++) {
        particles_[i].weight = peso_igual;
      }
    } else {
      for (size_t i = 0; i < particles_.size(); i++) {
        particles_[i].weight /= soma;
      }
    }
  }

  float ray_cast(double px, double py, double ang, float max_dist, double res, double ox, double oy, int w, int h) const
  {
    double passo = res / 2.0;  // avança meio pixel por vez
    double ca = std::cos(ang);
    double sa = std::sin(ang);

    double d = 0.0;
    while (d < max_dist) {
      d += passo;

      double vx = px + d * ca;
      double vy = py + d * sa;

      int c = (vx - ox) / res;
      int r = (vy - oy) / res;

      // saiu do mapa, considera que nao ha obstaculo
      if (c < 0 || c >= w || r < 0 || r >= h) {
        return max_dist;
      }

      int idx = r * w + c;
      if (idx >= 0 && idx < (int)current_map_->data.size()) {
        int8_t celula_raw = current_map_->data[idx];
        int celula = static_cast<int>(celula_raw);
        if (celula >= lim_livre) {
          return static_cast<float>(d); // encontrou obstáculo
        }
      }
    }
    return max_dist;
  }

  // RESAMPLING
  // Gera um novo conjunto de partículas dando mais chance para as de maior peso.
  void resample_particles()
  {
    int num = particles_.size();
    if (num == 0) 
      return;

    std::vector<Particle> novas;
    novas.reserve(num);

    std::uniform_real_distribution<double> sorteio(0.0, 1.0 / num);
    double r = sorteio(rng_);

    double c = particles_[0].weight;
    int idx = 0;

    // Percorre a roleta em passos de 1/N
    for (int m = 0; m < num; m++) {
      double u = r + (double)m / num;

      // Avança até encontrar a partícula que cobre o ponteiro u
      while (u > c && idx < num - 1) {
        idx++;
        c += particles_[idx].weight;
      }

      Particle p = particles_[idx];
      p.weight = 1.0 / num;
      novas.push_back(p);
    }

    particles_ = novas;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MCLNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}