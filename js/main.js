// Minimal JS: mobile nav toggle only (site is primarily static report)
document.addEventListener('DOMContentLoaded', ()=>{
  const menuBtn = document.getElementById('menuBtn');
  const mainNav = document.getElementById('mainNav');
  if(menuBtn && mainNav){
    menuBtn.addEventListener('click', ()=>mainNav.classList.toggle('show'));
  }
  // Smooth scroll for in-page links
  document.querySelectorAll('a[href^="#"]').forEach(a=>{
    a.addEventListener('click', (e)=>{
      const targetId = a.getAttribute('href').slice(1);
      if(!targetId) return;
      const target = document.getElementById(targetId);
      if(target){
        e.preventDefault();
        target.scrollIntoView({behavior:'smooth', block:'start'});
        // close mobile nav if open
        if(mainNav && mainNav.classList.contains('show')) mainNav.classList.remove('show');
      }
    });
  });
});