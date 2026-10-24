const links = [
  { label: 'GitHub', href: 'https://github.com/allenye66' },
  { label: 'LinkedIn', href: 'https://www.linkedin.com/in/allenye66/' },
]

function App() {
  return (
    <main>
      <h1>Allen Ye</h1>
      <p>Software engineer in Palo Alto.</p>
      <nav>
        {links.map((link) => (
          <a key={link.href} href={link.href}>
            {link.label}
          </a>
        ))}
      </nav>
    </main>
  )
}

export default App
