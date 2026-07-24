// Marsupial Simulator Project Website - JavaScript

document.addEventListener('DOMContentLoaded', function () {
    // Theme Toggle
    const themeToggle = document.getElementById('themeToggle');
    const prefersDark = window.matchMedia('(prefers-color-scheme: dark)').matches;

    const savedTheme = localStorage.getItem('theme');
    const initialTheme = savedTheme || (prefersDark ? 'dark' : 'light');

    document.documentElement.setAttribute('data-theme', initialTheme);

    themeToggle.addEventListener('click', function () {
        const currentTheme = document.documentElement.getAttribute('data-theme');
        const newTheme = currentTheme === 'dark' ? 'light' : 'dark';

        document.documentElement.setAttribute('data-theme', newTheme);
        localStorage.setItem('theme', newTheme);
    });

    // Smooth scroll
    document.querySelectorAll('a[href^="#"]').forEach(anchor => {
        anchor.addEventListener('click', function (e) {
            const href = this.getAttribute('href');
            if (href === '#') return;

            e.preventDefault();
            const target = document.querySelector(href);
            if (target) {
                const navHeight = document.querySelector('.navbar').offsetHeight;
                const targetPosition = target.getBoundingClientRect().top + window.pageYOffset - navHeight - 20;

                window.scrollTo({
                    top: targetPosition,
                    behavior: 'smooth'
                });

                // Close mobile menu if open
                const navLinks = document.getElementById('navLinks');
                const hamburger = document.getElementById('hamburgerBtn');
                if (navLinks && navLinks.classList.contains('open')) {
                    navLinks.classList.remove('open');
                    hamburger.classList.remove('active');
                }
            }
        });
    });

    // Segmented toggles (generic: works for any .segmented-toggle container)
    function initSegmentedToggle(containerId, checkboxId, indicatorId, panelIds) {
        const container = document.getElementById(containerId);
        const checkbox = document.getElementById(checkboxId);
        const indicator = document.getElementById(indicatorId);
        if (!container || !checkbox || !indicator) return;

        const options = container.querySelectorAll('.segment-option');
        const panels = panelIds.map(id => document.getElementById(id));
        if (options.length < 2 || panels.some(p => !p)) return;

        function updateIndicator(activeOption) {
            indicator.style.width = activeOption.offsetWidth + 'px';
            indicator.style.left = activeOption.offsetLeft + 'px';
        }

        function setActiveOption(index) {
            options.forEach(opt => opt.classList.remove('active-text'));
            panels.forEach(panel => panel.classList.remove('active'));
            options[index].classList.add('active-text');
            updateIndicator(options[index]);
            panels[index].classList.add('active');
        }

        setTimeout(() => updateIndicator(options[0]), 10);

        options.forEach((opt, index) => {
            opt.addEventListener('click', function () {
                checkbox.checked = index === 1;
                setActiveOption(index);
            });
        });

        checkbox.addEventListener('change', function () {
            setActiveOption(this.checked ? 1 : 0);
        });
    }

    initSegmentedToggle('resultsToggle', 'resultsViewToggle', 'resultsIndicator', ['panel-positions', 'panel-tether']);

    // ========================================
    // Image Carousel (abstract section)
    // ========================================
    const slides = document.querySelectorAll('.carousel-slide');
    const dots = document.querySelectorAll('.carousel-dot');
    const prevBtn = document.getElementById('carouselPrev');
    const nextBtn = document.getElementById('carouselNext');
    let currentSlide = 0;
    let autoplayTimer = null;

    function goToSlide(index) {
        if (slides.length === 0) return;
        slides[currentSlide].classList.remove('active');
        dots[currentSlide].classList.remove('active');
        currentSlide = (index + slides.length) % slides.length;
        slides[currentSlide].classList.add('active');
        dots[currentSlide].classList.add('active');

        // Update caption
        const captionObj = document.getElementById('carouselCaption');
        if (captionObj) {
            captionObj.textContent = slides[currentSlide].dataset.caption || '';
        }
    }

    function nextSlide() {
        goToSlide(currentSlide + 1);
    }

    function prevSlide() {
        goToSlide(currentSlide - 1);
    }

    function startAutoplay() {
        stopAutoplay();
        autoplayTimer = setInterval(nextSlide, 4500);
    }

    function stopAutoplay() {
        if (autoplayTimer) {
            clearInterval(autoplayTimer);
            autoplayTimer = null;
        }
    }

    if (prevBtn && nextBtn) {
        prevBtn.addEventListener('click', () => {
            prevSlide();
            startAutoplay(); // Reset autoplay on interaction
        });
        nextBtn.addEventListener('click', () => {
            nextSlide();
            startAutoplay();
        });
    }

    dots.forEach(dot => {
        dot.addEventListener('click', function () {
            goToSlide(parseInt(this.dataset.index));
            startAutoplay();
        });
    });

    // Start autoplay
    if (slides.length > 1) {
        startAutoplay();
    }

    // ========================================
    // Hamburger Mobile Menu
    // ========================================
    const hamburger = document.getElementById('hamburgerBtn');
    const navLinks = document.getElementById('navLinks');

    if (hamburger && navLinks) {
        hamburger.addEventListener('click', function () {
            this.classList.toggle('active');
            navLinks.classList.toggle('open');
        });

        // Close on outside click
        document.addEventListener('click', function (e) {
            if (navLinks.classList.contains('open') &&
                !navLinks.contains(e.target) &&
                !hamburger.contains(e.target)) {
                navLinks.classList.remove('open');
                hamburger.classList.remove('active');
            }
        });
    }

    // ========================================
    // Scroll-Reveal Animation
    // ========================================
    const revealElements = document.querySelectorAll('[data-reveal]');

    if (revealElements.length > 0) {
        const revealObserver = new IntersectionObserver((entries) => {
            entries.forEach(entry => {
                if (entry.isIntersecting) {
                    entry.target.classList.add('revealed');
                    revealObserver.unobserve(entry.target);
                }
            });
        }, {
            threshold: 0.12,
            rootMargin: '0px 0px -40px 0px'
        });

        revealElements.forEach(el => revealObserver.observe(el));
    }

    // ========================================
    // Lightbox
    // ========================================
    const lightboxOverlay = document.getElementById('lightboxOverlay');
    const lightboxImg = document.getElementById('lightboxImg');
    const lightboxClose = document.getElementById('lightboxClose');

    if (lightboxOverlay && lightboxImg) {
        // Make all result images and carousel images clickable
        const clickableImages = document.querySelectorAll('.images-row img, .carousel-slide img');

        clickableImages.forEach(img => {
            img.style.cursor = 'zoom-in';
            img.addEventListener('click', function () {
                lightboxImg.src = this.src;
                lightboxImg.alt = this.alt;
                lightboxOverlay.classList.add('active');
                document.body.style.overflow = 'hidden';
            });
        });

        function closeLightbox() {
            lightboxOverlay.classList.remove('active');
            document.body.style.overflow = '';
        }

        lightboxClose.addEventListener('click', closeLightbox);
        lightboxOverlay.addEventListener('click', function (e) {
            if (e.target === this) closeLightbox();
        });

        document.addEventListener('keydown', function (e) {
            if (e.key === 'Escape' && lightboxOverlay.classList.contains('active')) {
                closeLightbox();
            }
        });
    }

    // ========================================
    // Copy buttons (code blocks & citation)
    // ========================================
    const copyBtns = document.querySelectorAll('.code-block .copy-btn');

    copyBtns.forEach(btn => {
        btn.addEventListener('click', async function () {
            const block = this.closest('.code-block');
            const codeContent = block.querySelector('.copy-content').textContent;

            try {
                await navigator.clipboard.writeText(codeContent);
                const copyText = this.querySelector('.copy-text');
                copyText.textContent = 'Copied!';
                this.classList.add('copied');

                setTimeout(() => {
                    copyText.textContent = 'Copy';
                    this.classList.remove('copied');
                }, 2000);
            } catch (err) {
                console.error('Failed to copy:', err);
            }
        });
    });
});
