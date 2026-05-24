module tb_driver_crc();

  sdhci_sd_driver driver (
    .sd_clk_i('0),
    .sd_cmd_o(),
    .sd_cmd_i('0),
    .sd_cmd_en_i('0),
    .sd_dat_o(),
    .sd_dat_i('0),
    .sd_dat_en_i('0)
  );

  function check(
    logic [4095:0] data,
    int data_length,
    logic [15:0] expected
  );

    logic [15:0] actual;
    actual = driver.calculate_crc16(
      .data(data),
      .data_length(data_length)
    );

    if (actual != expected) begin
      $fatal(1, "Expected a CRC of %04x, but got %04x, input was %x", expected, actual, data);
    end
  endfunction


  initial begin
    check(
      .data('0),
      .data_length(4096),
      .expected('0)
    );
    check(
      .data('h1),
      .data_length(4096),
      .expected('h1021)
    );

    check( // polynomial should give 0
      .data('h011021),
      .data_length(17),
      .expected('0)
    );

    check(
      .data('h01c2b04ae3c8d7f821a752c1e43a15),
      .data_length(30*4),
      .expected('h4716)
    );

    check(
      .data('he9538155b17a78835f930faeff4f65dff886b379ede29d835e86015b4fbcdee866822c04261070d21c491dde1e5ccc34f1f0a62cf7966f5cfaba1ba46281dd8623732c90982f9dc046d259aab17e77926c2718f47e038bd8fb0cfa30d790481ac80f8ae8a4665465489d7e0e6457a0b3d669ca263f12509e328a21a0e831a3ed5a940f652802a7b665d7cdfdcfef946ae7944be2de8002d6c5e5d48c1556331fba36f821560a29b9aef06bf1f071da91acba8b0facd46c1b2508ee70e829536c8ad130d2148b647490af8ba84321ef11e74a48933215504eacc79d4e10757e537ed26c7664d7cb995fefec69203c5ed1364f5c19f1796e7568b1e93cfb7169493ed34abeb03d1187ec0e4f98710ee00952c9c624bbaac4b73af0411305decb65330b80eaedd778e1f30a71f9804557664585468373c5d00e0d2915ffa086622d7f49f6b956e1d7f0bee04d4ac7ec3c4c0ede48a4da72a92fb73dfd6322deaee6d90910b8b512cbb92183929d6e4262f5959d6352bff08eaa211a2f292f3e8450deb41c74eee2f906918acebf8d7429900cad00c5209c82c2f5aea210a3da5d900ffdefeeacc1da3a3fc4300ae247de110a06e2f8c4d997d2139e155cafba526ae8f85c50cfd144055a285e745b6559f8e1902400f019926ac6b2f42f5282b1b9ddbd7ab4887d091561650f37d1d87500b1e6685054dbc7b297bdc4bbd52daa67),
      .data_length(4096),
      .expected('hc0b2)
    );

    check(
      .data('h353aaa63f3784d51aaf19e5c1b21fb464109ae73d33355bdb158b3605bf4d97686124dbe0b961ea03eef268be29231ae8f6deeb03a956e75c50a6b98fba01c5673094853f23c678d5fed22f33fee1e40f38418d103a9afb6cfe12d63ffeba31fc63c471dc91b1959bd99f6bf973429a9b03e15bdbba57c2075e60faf7de10264ba6a9ce9d5436fde5075f14baa511cc4e9a3ccc22de174d4b478f5bf931f2ad06c9625c11f803cb8894740fb9875ee1378708b956e3bde768ad97cc51562bab3091a7fb55ccfb942646d41de50656955838e77b6ee9a18e85e6025d2d185f7eff3a345ddde50607d00ba68b09f402aff7999e279f0209152c43a624e1da88ece0fe47d56b08a1feba14f425146c81fd4cf16ef90f8300cbe37e45f0796d5cadd4d26a607605b10494adc93ecbfb003efe70ebfc0e6b52dc88f3cf7fd5c0b68e9923ba6fb30f7e4cf0762fb635c1a0e3c07cee7b4cf7014623c709cda2fb0618417d95678bc762abf928621d8df8a071f0f05c08d797c46ee055bf039888c86870535900413fb9dab8aacadcd14802b62d55e89d74687edf06e7cb5d818ca06e68e88ab62c1533fc03da7be1ba58530dd632943cb9b0765f9bf49d951487d5048a17e964396ad8996955ea64a47b6424264ac829d474433257ddbd652ce335445847d7721f54b05b55eed1465004a445bdac9749f824fb116bcc426b35b712015),
      .data_length(4096),
      .expected('h8dc2)
    );


    $display("All good");
    $finish();
  end

endmodule
